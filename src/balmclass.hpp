#ifndef BALMCLASS
#define BALMCLASS

#include <ros/ros.h>
#include <pcl/common/transforms.h>
#include <unordered_map>
#include <opencv2/opencv.hpp>
#include "myso3/myso3.hpp"
#include <thread>
#include <mutex>
#include <fstream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
typedef std::vector<Eigen::Vector3d> PL_VEC;
typedef pcl::PointXYZINormal PointTypeXYZIN;
#define MIN_PS 7
using namespace std;

// Key of hash table
//哈希表的键 root voxel索引
class VOXEL_LOC
{
public:
  //在地图上的索引坐标
  int64_t x, y, z;

  //构造函数
  VOXEL_LOC(int64_t vx=0, int64_t vy=0, int64_t vz=0): x(vx), y(vy), z(vz){}

  //==运算符重载
  bool operator== (const VOXEL_LOC &other) const
  {
    return (x==other.x && y==other.y && z==other.z);
  }
};

// Hash value
//重载哈希函数
namespace std
{
  template<>
  struct hash<VOXEL_LOC>
  {
    size_t operator() (const VOXEL_LOC &s) const
    {
      using std::size_t; using std::hash;
      return ((hash<int64_t>()(s.x) ^ (hash<int64_t>()(s.y) << 1)) >> 1) ^ (hash<int64_t>()(s.z) << 1);
    }
  };
}

//一个体素的数据结构，xyz是点坐标的累加 count是点的个数
struct M_POINT
{
  float xyz[3];
  int count = 0;
};

// Similar with PCL voxelgrid filter
/**
 * @brief 输入一帧点云，计算每个点所在的体素索引，输入点云清空后再把每个体素中的点云的平均中心点存入输入点云
 * 
 * @param pl_feat 输入点云 最终存储哈希表中每个体素中点云的中心坐标
 * @param voxel_size 体素大小
 */
void down_sampling_voxel(pcl::PointCloud<PointTypeXYZIN> &pl_feat, double voxel_size)
{
  if(voxel_size < 0.01)
  {
    return;
  }

  //哈希表，键是体素索引，值是体素数据结构
  unordered_map<VOXEL_LOC, M_POINT> feat_map;
  //统计点云中点数
  uint plsize = pl_feat.size();
  //遍历点云
  for(uint i=0; i<plsize; i++)
  {
    //取出一点
    PointTypeXYZIN &p_c = pl_feat[i];
    float loc_xyz[3];
    //计算root voxel索引
    for(int j=0; j<3; j++)
    {
      loc_xyz[j] = p_c.data[j] / voxel_size;
      if(loc_xyz[j] < 0)
      {
        loc_xyz[j] -= 1.0;
      }
    }

    //利用计算出的root voxel索引构建哈希表的键
    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
    //在哈希表中查找该键
    auto iter = feat_map.find(position);
    //如果找到了，说明已经有这个索引的root voxel了
    if(iter != feat_map.end())
    {
      //累加点坐标
      iter->second.xyz[0] += p_c.x;
      iter->second.xyz[1] += p_c.y;
      iter->second.xyz[2] += p_c.z;
      //每添加一个点，count++
      iter->second.count++;
    }
    //如果每没找到，说明这是第一个落在这个体素的点
    else
    {
      //构建体素数据结构
      M_POINT anp;
      //赋值第一个点的坐标
      anp.xyz[0] = p_c.x;
      anp.xyz[1] = p_c.y;
      anp.xyz[2] = p_c.z;
      //点的个数为1
      anp.count = 1;
      //在哈希表中添加数据
      feat_map[position] = anp;
    }
  }

  //当前哈希表中体素数量
  plsize = feat_map.size();
  //清空输入点云
  pl_feat.clear();
  //resize为体素数量
  pl_feat.resize(plsize);
  
  uint i = 0;
  //遍历哈希表
  for(auto iter=feat_map.begin(); iter!=feat_map.end(); ++iter)
  {
    //计算一个体素中坐标的平均值，也就是该体素当前的中心/质心
    pl_feat[i].x = iter->second.xyz[0]/iter->second.count;
    pl_feat[i].y = iter->second.xyz[1]/iter->second.count;
    pl_feat[i].z = iter->second.xyz[2]/iter->second.count;
    i++;
  }

}

/**
 * @brief 输入一帧点云，计算每个点所在的体素索引，输入点云清空后再把每个体素中的点云的中心点存入输入点云
 *        和上一个函数的区别在于voxel_size不必小于0.01 还有输入点云的存储形式，这里是vector形式存储的点云
 * @param pl_feat 容器形式存储的点云
 * @param voxel_size 体素大小
 */
void down_sampling_voxel(PL_VEC &pl_feat, double voxel_size)
{
  //构建哈希表
  unordered_map<VOXEL_LOC, M_POINT> feat_map;
  //统计点云点数
  uint plsize = pl_feat.size();
  //遍历点云
  for(uint i=0; i<plsize; i++)
  {
    //取一点
    Eigen::Vector3d &p_c = pl_feat[i];
    double loc_xyz[3];
    for(int j=0; j<3; j++)
    {
      //计算体素索引
      loc_xyz[j] = p_c[j] / voxel_size;
      if(loc_xyz[j] < 0)
      {
        loc_xyz[j] -= 1.0;
      }
    }
    //构建哈希表的键
    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
    auto iter = feat_map.find(position);
    if(iter != feat_map.end())
    {
      iter->second.xyz[0] += p_c[0];
      iter->second.xyz[1] += p_c[1];
      iter->second.xyz[2] += p_c[2];
      iter->second.count++;
    }
    else
    {
      M_POINT anp;
      anp.xyz[0] = p_c[0];
      anp.xyz[1] = p_c[1];
      anp.xyz[2] = p_c[2];
      anp.count = 1;
      feat_map[position] = anp;
    }

  }
  //统计体素个数
  plsize = feat_map.size();
  pl_feat.resize(plsize);

  uint i = 0;
  for(auto iter=feat_map.begin(); iter!=feat_map.end(); ++iter)
  {
    //计算每个体素的中心点坐标
    //第i个体素的x、y、z坐标
    pl_feat[i][0] = iter->second.xyz[0]/iter->second.count;
    pl_feat[i][1] = iter->second.xyz[1]/iter->second.count;
    pl_feat[i][2] = iter->second.xyz[2]/iter->second.count;
    i++;
  }

}

/**
 * @brief 坐标变换
 * 
 * @param orig 原始点云
 * @param tran 变换后点云
 * @param R 原始点云坐标系到新点云坐标系的旋转矩阵
 * @param t 原始点云坐标系到新点云坐标系的平移向量
 */
void plvec_trans_func(vector<Eigen::Vector3d> &orig, vector<Eigen::Vector3d> &tran, Eigen::Matrix3d R, Eigen::Vector3d t)
{
  //统计原始点云个数
  uint orig_size = orig.size();
  tran.resize(orig_size);

  for(uint i=0; i<orig_size; i++)
  {
    tran[i] = R*orig[i] + t;
  }
}

/**
 * @brief 发布点云
 * 
 * @tparam T 
 * @param pl 待发布的点云
 * @param pub 发布器
 * @param current_time 时间戳 
 */
template <typename T>
void pub_func(T &pl, ros::Publisher &pub, const ros::Time &current_time)
{
  pl.height = 1; pl.width = pl.size();
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(pl, output);
  output.header.frame_id = "camera_init";
  output.header.stamp = current_time;
  pub.publish(output);
}

// Convert PointCloud2 to PointTypeXYZIN
/**
 * @brief ros点云消息转pcl点云消息 这里主要是plt多了一个normal通道
 * @param pl_msg 
 * @param plt 
 */
void rosmsg2ptype(const sensor_msgs::PointCloud2 &pl_msg, pcl::PointCloud<PointTypeXYZIN> &plt)
{
  pcl::PointCloud<pcl::PointXYZI> pl;
  pcl::fromROSMsg(pl_msg, pl);

  uint asize = pl.size();
  plt.resize(asize);

  for(uint i=0; i<asize; i++)
  {
    plt[i].x = pl[i].x;
    plt[i].y = pl[i].y;
    plt[i].z = pl[i].z;
    plt[i].intensity = pl[i].intensity;
  }
}

// Convert PointTypeXYZI to PointTypeXYZIN
/**
 * @brief pcl格式转换
 * @param pl_msg 
 * @param plt 
 */
void ptypeXYZI2ptypeXYZIN(pcl::PointCloud<PointType> &pls, pcl::PointCloud<PointTypeXYZIN> &plt)
{
  uint asize = pls.size();
  plt.resize(asize);

  for(uint i=0; i<asize; i++)
  {
    plt[i].x = pls[i].x;
    plt[i].y = pls[i].y;
    plt[i].z = pls[i].z;
    plt[i].intensity = pls[i].intensity;
  }
}

/**
 * @brief 把float[6]转化为eigen四元数和vector3d
 * 
 */
void transformTobeMappedToQuaternionAndVector(const float transformTobeMapped[6], Eigen::Quaterniond& quaternion, Eigen::Vector3d& translation) {
  double roll = transformTobeMapped[0];
  double pitch = transformTobeMapped[1];
  double yaw = transformTobeMapped[2];

  double x = transformTobeMapped[3];
  double y = transformTobeMapped[4];
  double z = transformTobeMapped[5];

  quaternion = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
              Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  translation << x, y, z;
}

/**
 * @brief 反向转换
 * 
 */
void quaternionAndVectorToTransformTobeMapped(const Eigen::Quaterniond& quaternion, const Eigen::Vector3d& translation, float transformTobeMapped[6]) {
  Eigen::Vector3d euler_angles = quaternion.toRotationMatrix().eulerAngles(2, 1, 0);
  transformTobeMapped[0] = euler_angles(2);
  transformTobeMapped[1] = euler_angles(1);
  transformTobeMapped[2] = euler_angles(0);
  transformTobeMapped[3] = translation.x();
  transformTobeMapped[4] = translation.y();
  transformTobeMapped[5] = translation.z();
}

// P_fix in the paper
// Summation of P_fix
//体素边缘化用到的类
class SIG_VEC_CLASS
{
public:
  //滑窗外的点构成的A矩阵
  Eigen::Matrix3d sigma_vTv;
  //滑窗外的点的中心点 也就是P_fix
  Eigen::Vector3d sigma_vi;
  //边缘化掉的帧 降采样后中心点数量
  int sigma_size;

  //构造函数
  SIG_VEC_CLASS()
  {
    sigma_vTv.setZero();
    sigma_vi.setZero();
    sigma_size = 0;
  }

  //清空数据结构
  void tozero()
  {
    sigma_vTv.setZero();
    sigma_vi.setZero();
    sigma_size = 0;
  }

};

const double one_three = (1.0 / 3.0);
//拟合面 最大特征值/最小特征值 拟合线 最大特征值/第二大特征值 阈值，如果大于该阈值说明特征拟合成功
double feat_eigen_limit[2] = {3*3, 2*2};
//?这个参数是用于第二次拟合 所以更加严格
//拟合面 最大特征值/最小特征值 拟合线 最大特征值/第二大特征值 阈值，如果大于该阈值说明特征拟合成功
double opt_feat_eigen_limit[2] = {4*4, 3*3};

// LM optimizer for map-refine
// 滑窗LM优化器 滑窗是存在于体素中的 每个voxel都有一个滑窗优化器 这个类是一个大的滑窗优化器，里面要存储多个voxel的滑窗，所有滑窗都有相同的大小，求解的姿态是唯一的，也就是说多个滑窗共同构建优化问题，但只求解滑窗大小个数的位姿
class LM_SLWD_VOXEL
{
public: 
  //滑窗大小  体素滑窗内每个关键帧的点数？？ 滑窗优化时计算矩阵线程数量 计算Jacobian Hessian矩阵的多线程数 雅克比矩阵维度
  int slwd_size, filternum, thd_num, jac_leng;
  //最高迭代次数
  int iter_max = 20;
  //?构造函数中赋值
  double corn_less;

  //构造函数中全部设置为滑窗大小
  //滑窗中每个关键帧的旋转姿态 在世界坐标系下 所有体素共用一个滑窗关键帧的位姿
  vector<SO3> so3_poses, so3_poses_temp;
  //滑窗中每个关键帧的位置 在世界坐标系下
  vector<Eigen::Vector3d> t_poses, t_poses_temp;

  //拟合的特征种类 0代表平面 1代表直线 每个元素对应一个体素滑窗
  vector<int> lam_types; // 0 surf, 1 line
  //边缘化的残留数据 每个元素对应一个体素滑窗
  vector<SIG_VEC_CLASS> sig_vecs;
  //存储每个体素滑窗中所有关键帧的点坐标，每个元素对应一个体素滑窗
  vector<vector<Eigen::Vector3d>*> plvec_voxels;
  //存储每个体素滑窗中所有点所在关键帧在滑窗中的索引，和点数量对应 每个元素对应一个体素滑窗
  vector<vector<int>*> slwd_nums;
  //滑窗优化标志
  int map_refine_flag;
  mutex my_mutex;

  //构造函数
  LM_SLWD_VOXEL(int ss, int fn, int thnum): slwd_size(ss), filternum(fn), thd_num(thnum)
  {
    //全部设置为滑窗大小
    so3_poses.resize(ss); t_poses.resize(ss);
    so3_poses_temp.resize(ss); t_poses_temp.resize(ss);
    //雅克比矩阵维度 滑窗中有M个scan 6M 这是公式13的大J矩阵
    jac_leng = 6*ss;
    //?
    corn_less = 0.1;
    //没有进行滑窗优化
    map_refine_flag = 0;
  }

  // Used by "push_voxel"
  /**
   * @brief 把一个关键帧中的点和该关键帧在滑窗中的索引记录下来
   * 
   * @param plvec_orig 当前关键帧内的点坐标 在当前关键帧坐标系下表示  原始帧点坐标
   * @param cur_frame 当前关键帧在滑窗中的索引 从0~slwd_size
   * @param plvec_voxel 存入点坐标  存入到该体素内的点坐标作为输出
   * @param slwd_num 存入当前关键帧在滑窗中的索引 每个元素对应存入的每一个点
   * @param filternum2use 给这个关键帧预留的内存空间的大小
   */
  void downsample(vector<Eigen::Vector3d> &plvec_orig, int cur_frame,vector<Eigen::Vector3d> &plvec_voxel, vector<int> &slwd_num, int filternum2use)
  {
    //统计当前关键帧点个数
    uint plsize = plvec_orig.size();
    //如果点个数小于预留的空间，说明预留的空间足够，则开始存储
    if(plsize <= (uint)filternum2use)
    {
      //遍历每个点
      for(uint i=0; i<plsize; i++)
      {
        //存入点坐标
        plvec_voxel.push_back(plvec_orig[i]);
        //存入当前点所在的关键帧在滑窗中的索引
        slwd_num.push_back(cur_frame);
      }
      return;
    }

    //如果预留的空间不足 类似把点降采样一下，几个点求一个均值点存进去
    Eigen::Vector3d center;
    //原始一共plsize个点，要尽量分成filternum2use个
    double part = 1.0 * plsize / filternum2use;

    for(int i=0; i<filternum2use; i++)
    {
      uint np = part*i;
      uint nn = part*(i+1);
      center.setZero();
      //每份计算均值点坐标
      for(uint j=np; j<nn; j++)
      {
        center += plvec_orig[j];
      }
      center = center / (nn-np);
      //存入均值点
      plvec_voxel.push_back(center);
      //存入对应关键帧在滑窗中的索引
      slwd_num.push_back(cur_frame);
    }
  }

  // Push voxel into optimizer
  /**
   * @brief 把一个体素滑窗中的所有关键帧的点云和当前体素的边缘化残留的信息传递给滑窗优化器
   * 
   * @param plvec_orig 一个体素滑窗中的多个关键帧的点云
   * @param sig_vec 该体素边缘化的数据
   * @param lam_type 拟合特征 0-平面 1-直线
   */
  void push_voxel(vector<vector<Eigen::Vector3d>*> &plvec_orig, SIG_VEC_CLASS &sig_vec, int lam_type)
  {
    //统计体素滑窗中有几个有效的关键帧
    int process_points_size = 0;
    //遍历体素滑窗
    for(int i=0; i<slwd_size; i++)
    {
      //统计体素滑窗中有几个有效的关键帧
      if(!plvec_orig[i]->empty())
      {
        process_points_size++;
      }
    }
    
    // Only one scan
    //如果只有一个有效关键帧或者0帧，直接返回
    // if(process_points_size <= 1)
    // {
    //   //todo 每次都走到这里 导致矩阵都是空的
    //   ROS_INFO("push_voxel inside mark");
    //   return;
    // }

    //filternum2use是留出的每个关键帧的点数
    int filternum2use = filternum;
    //空间也不能太少，至少保证每个关键帧有一个位置
    if(filternum*process_points_size < MIN_PS)
    {
      filternum2use = MIN_PS / process_points_size + 1;
    }

    //当前体素中的所有点坐标
    vector<Eigen::Vector3d> *plvec_voxel = new vector<Eigen::Vector3d>();
    // Frame num in sliding window for each point in "plvec_voxel"
    //存储当前体素中所有点所在关键帧在滑窗中的索引 每个元素对应一个点
    vector<int> *slwd_num = new vector<int>();
    //预留空间，每个关键帧点数*滑窗中有效关键帧帧数=预留的体素滑窗内包含的所有点数
    plvec_voxel->reserve(filternum2use*slwd_size);
    slwd_num->reserve(filternum2use*slwd_size);

    // retain one point for one scan (you can modify)
    //遍历体素滑窗中每个关键帧
    for(int i=0; i<slwd_size; i++)
    {
      //如果当前关键帧是有效的
      if(!plvec_orig[i]->empty())
      {
        downsample(*plvec_orig[i], i, *plvec_voxel, *slwd_num, filternum2use);
      }
    }

    // for(int i=0; i<slwd_size; i++)
    // {
    //   for(uint j=0; j<plvec_orig[i]->size(); j++)
    //   {
    //     plvec_voxel->push_back(plvec_orig[i]->at(j));
    //     slwd_num->push_back(i);
    //   }
    // }

    // ROS_INFO("push_voxel inside mark");

    //把当前体素的滑窗中所有点 放入优化器
    plvec_voxels.push_back(plvec_voxel); // Push a voxel into optimizer
    //把当前体素的滑窗中所有点对应的关键帧在滑窗中的索引存下来 放入优化器
    slwd_nums.push_back(slwd_num);
    //存储当前体素滑窗优化的拟合特征类型
    lam_types.push_back(lam_type);
    //存储当前体素滑窗优化的边缘化残留数据
    sig_vecs.push_back(sig_vec); // history points out of sliding window
  }

  // Calculate Hessian, Jacobian, residual
  /**
   * @brief 
   * 
   * @param so3_ps 所有体素滑窗共用的每个关键帧的旋转姿态 在世界坐标系下
   * @param t_ps 所有体素滑窗共用的每个关键帧的位移 在世界坐标系下
   * @param head 要参与计算的体素滑窗的起始索引 左闭右开区间
   * @param end 要参与计算的体素滑窗的终止索引
   * @param Hess 计算得到的大Hessian矩阵（也可能是一部分，最后要累加起来）
   * @param JacT 计算得到的大Jacobian矩阵（也可能是一部分，最后要累加起来）
   * @param residual 计算得到的残差（也可能是一部分，最后要累加起来）
   */
  void acc_t_evaluate(vector<SO3> &so3_ps, vector<Eigen::Vector3d> &t_ps, int head, int end, Eigen::MatrixXd &Hess, Eigen::VectorXd &JacT, double &residual)
  {
    //最终结果的大矩阵初始化
    Hess.setZero(); JacT.setZero(); residual = 0;
    //下面两个变量是函数内计算用的 存本次计算的大H和大J转置的矩阵 最终都累加到传进来的Hess 和 JacT中
    //论文中公式14的大H矩阵
    Eigen::MatrixXd _hess(Hess);
    //6M x 1的矩阵 和论文公式14的大J矩阵差个转置
    Eigen::MatrixXd _jact(JacT);

    // In program, lambda_0 < lambda_1 < lambda_2
    // For plane, the residual is lambda_0
    // For line, the residual is lambda_0+lambda_1
    // We only calculate lambda_1 here
    //遍历每个要计算的体素滑窗 最终结果的大H 大J矩阵综合了所有体素滑窗
    for(int a=head; a<end; a++)
    {
      //当前体素滑窗要拟合的特征类型 0代表平面 1代表直线
      uint k = lam_types[a]; // 0 is surf, 1 is line
      //取出边缘化的残留数据
      SIG_VEC_CLASS &sig_vec = sig_vecs[a];
      //把当前体素滑窗中所有点坐标取出来
      vector<Eigen::Vector3d> &plvec_voxel = *plvec_voxels[a];
      // Position in slidingwindow for each point in "plvec_voxel"
      //取出当前体素滑窗中所有点所在关键帧在滑窗中的索引 每个元素对应一个点
      vector<int> &slwd_num = *slwd_nums[a]; 
      //统计当前体素滑窗中点的数量
      uint backnum = plvec_voxel.size();

      //用于点在世界坐标系下的坐标
      Eigen::Vector3d vec_tran;
      //存储所有点在世界坐标系下的坐标
      vector<Eigen::Vector3d> plvec_back(backnum);
      // derivative point to T (R, t)
      vector<Eigen::Matrix3d> point_xis(backnum);
      //均值
      Eigen::Vector3d centor(Eigen::Vector3d::Zero());
      //协方差矩阵
      Eigen::Matrix3d covMat(Eigen::Matrix3d::Zero());

      //遍历当前体素滑窗中的所有点
      for(uint i=0; i<backnum; i++)
      {
        //把点坐标转换到世界坐标系下
        //对应公论文式10的 R_si * P_fi
        vec_tran = so3_ps[slwd_num[i]].matrix() * plvec_voxel[i];
        // left multiplication instead of right muliplication in paper
        //point_xis是 -(R_si * P_fi)^{hat} 是左乘扰动模型中 变换后点坐标对位姿求导的旋转部分的雅克比矩阵
        point_xis[i] = -SO3::hat(vec_tran);
        plvec_back[i] = vec_tran + t_ps[slwd_num[i]]; // after trans

        //当前体素滑窗内点在世界系下坐标累加 计算均值和协方差矩阵
        centor += plvec_back[i];
        covMat += plvec_back[i] * plvec_back[i].transpose();
      }

      //这部分就是把当前体素滑窗边缘化掉的部分也算进来 计算均值和协方差矩阵
      //统计当前体素滑窗内的点和边缘化的点的总数
      double N_points = backnum + sig_vec.sigma_size;
      //把边缘化留下的点累加进来
      centor += sig_vec.sigma_vi;
      covMat += sig_vec.sigma_vTv;
      //计算最后的均值和协方差矩阵
      covMat = covMat - centor*centor.transpose()/N_points;
      covMat = covMat / N_points;
      centor = centor / N_points;

      //计算特征值和特征向量
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);
      Eigen::Vector3d eigen_value = saes.eigenvalues();

      Eigen::Matrix3d U = saes.eigenvectors();
      //这是数组 存三个特征向量
      Eigen::Vector3d u[3]; // eigenvectors
      //按照特征值从小到大排列的顺序 存下特征向量
      for(int j=0; j<3; j++)
      {
        u[j] = U.block<3, 1>(0, j);
      }

      // Jacobian matrix
      //面特征k=0 线特征k=1 k的值对应论文中的公式2和4
      Eigen::Matrix3d ukukT = u[k] * u[k].transpose();
      //论文公式6的雅克比矩阵 和论文中稍有不同 求的是公式6的转置
      Eigen::Vector3d vec_Jt;
      //遍历当前体素滑窗中所有点
      for(uint i=0; i<backnum; i++)
      {
        //世界系下点坐标-（体素滑窗内点和边缘化点的均值）
        plvec_back[i] = plvec_back[i] - centor;
        //论文中公式6雅可比矩阵的转置
        vec_Jt = 2.0/N_points * ukukT * plvec_back[i];
        //slwd_num是当前点所在关键帧在滑窗中的索引
        //这里为什么是累加 因为每个和这个关键帧位姿相关的点都对最小二乘问题有贡献
        //因为计算的是论文公式14中的大J矩阵的转置 所以用公式6的J矩阵的转置 乘以 单位矩阵的转置
        _jact.block<3, 1>(6*slwd_num[i]+3, 0) += vec_Jt;
        //point_xis是 -(R_si * P_fi)^{hat}
        //反对称矩阵的转置是原矩阵加负号 所以这里用-= 其实是累加
        _jact.block<3, 1>(6*slwd_num[i], 0) -= point_xis[i] * vec_Jt;
      }

      // Hessian matrix
      //这个变量是存计算中间量 会存不同的值
      Eigen::Matrix3d Hessian33;
      //对应论文公式7第二个公式 F^{P_j}_{k} 3x3矩阵
      Eigen::Matrix3d C_k;
      //对应公式7的最后一个公式 因为m要取1，2，3，所以定义为vector 定义为Matrix3d的原因是还没乘以p_j - p^{bar}的转置
      vector<Eigen::Matrix3d> C_k_np(3);
      for(uint i=0; i<3; i++)
      {
        //k:0平面 1:直线
        //如果拟合平面特征，
        if(i == k)
        {
          C_k_np[i].setZero();
          continue;
        }
        //这里对应论文公式7最后一个公式的u_n x u_m
        Hessian33 = u[i]*u[k].transpose();
        // part of F matrix in paper
        //对应论文公式7最后一个公式
        //i对应论文中m k对应论文中的n 这个是3x3矩阵 这里的负号是因为eigen_value[i]-eigen_value[k]
        C_k_np[i] = -1.0/N_points/(eigen_value[i]-eigen_value[k])*(Hessian33 + Hessian33.transpose());
      }

      //计算中间量
      Eigen::Matrix3d h33;
      //记录中心点对应大H矩阵中的位置
      uint rownum, colnum;
      //遍历当前体素滑窗中所有点
      for(uint j=0; j<backnum; j++)
      {
        for(int f=0; f<3; f++)
        {
          //对应论文公式7 F^{P_j}_{k} 3x3矩阵
          //这个是1x3矩阵 对应论文公式7的最后一个公式
          C_k.block<1, 3>(f, 0) = plvec_back[j].transpose() * C_k_np[f];
        }
        //从这里开始计算公式7的第一个公式
        C_k = U * C_k;
        //一个关键帧6自由度位姿
        colnum = 6*slwd_num[j];
        // block matrix operation, half Hessian matrix
        //只算大Hessian矩阵的一半 因为Hessian矩阵是对称矩阵，后面另一半直接复制过去
        for(uint i=j; i<backnum; i++)
        {
          //plvec_back[i]就是p_i - p^{bar}
          //对应论文公式7第一个公式的第二项 + 第三项 第三项的计算中u[k].dotplvec_back[i]结果是数字 所以可以换位置 和论文有一点不同
          Hessian33 = u[k]*(plvec_back[i]).transpose()*C_k + u[k].dot(plvec_back[i])*C_k;
          //当前两个点对应的矩阵块在大H矩阵（H^{bar}）中的索引 是一个6X6矩阵块 计算的是矩阵块左上角的位置索引
          rownum = 6*slwd_num[i];
          //对应论文公式7 两种情况 这里还没乘以 2/N
          if(i == j)
          {
            Hessian33 += (N_points-1)/N_points * ukukT;
          }
          else
          {
            Hessian33 -= 1.0/N_points * ukukT;
          }
          //最后都要乘以2/N 至此论文公式7求出来了
          Hessian33 = 2.0/N_points * Hessian33; // Hessian matrix of lambda and point

          // Hessian matrix of lambda and pose
          //?这里细节还没对应上 但确实是求大H矩阵
          if(rownum==colnum && i!=j)
          {
            _hess.block<3, 3>(rownum+3, colnum+3) += Hessian33 + Hessian33.transpose();

            h33 = -point_xis[i]*Hessian33;
            _hess.block<3, 3>(rownum, colnum+3) += h33;
            _hess.block<3, 3>(rownum+3, colnum) += h33.transpose();
            h33 = Hessian33*point_xis[j];
            _hess.block<3, 3>(rownum+3, colnum) += h33;
            _hess.block<3, 3>(rownum, colnum+3) += h33.transpose();
            h33 = -point_xis[i] * h33;
            _hess.block<3, 3>(rownum, colnum) += h33 + h33.transpose();
          }
          //?
          else
          {
            _hess.block<3, 3>(rownum+3, colnum+3) += Hessian33;
            h33 = Hessian33*point_xis[j];
            _hess.block<3, 3>(rownum+3, colnum) += h33;
            //注意下面两个都是-= 还是和反对称矩阵的转置有关
            _hess.block<3, 3>(rownum, colnum+3) -= point_xis[i]*Hessian33;
            _hess.block<3, 3>(rownum, colnum) -= point_xis[i]*h33;
          }
        }
      }

      //到这里要参与计算的一个体素滑窗中的关键帧信息都处理完了 可以累加到最终结果的大H矩阵上了
      //在大H矩阵上添加阻尼因子 对应论文公式14
      //平面的residual就是最小的特征值
      //k:0平面 1:直线
      if(k == 1)
      {
        // add weight for line feature
        residual += corn_less*eigen_value[k];
        Hess += corn_less*_hess; JacT += corn_less*_jact;
      }
      //直线的residual其实是最小的两个特征值相加 这里只算进来第二大的特征值
      else
      {
        residual += eigen_value[k];
        Hess += _hess; JacT += _jact;
      }
      _hess.setZero(); _jact.setZero();
    }

    // Hessian is symmetric, copy to save time
    //对称的部分复制过去
    for(int j=0; j<jac_leng; j+=6)
    {
      for(int i=j+6; i<jac_leng; i+=6)
      {
        Hess.block<6, 6>(j, i) = Hess.block<6, 6>(i, j).transpose();
      }
    }
  }

  // Multithread for "acc_t_evaluate"
  /**
   * @brief 
   * 
   * @param so3_ps 滑窗中每个关键帧的旋转姿态 在世界坐标系下
   * @param t_ps 滑窗中每个关键帧的位移 在世界坐标系下
   * @param Hess 计算得到的Hessian矩阵 6M x 6M
   * @param JacT 计算得到的Jacobian矩阵 6M x 1
   * @param residual 计算得到的残差
   */
  void divide_thread(vector<SO3> &so3_ps, vector<Eigen::Vector3d> &t_ps,Eigen::MatrixXd &Hess, Eigen::VectorXd &JacT, double &residual)
  {
    // ROS_INFO("divide_thread inside mark 1");
    //初始化
    //Hess:6M x 6M J^T:6M x 1
    Hess.setZero(); JacT.setZero(); residual = 0;

    //一共要计算thd_num个矩阵
    //算多个的原因是，把各部分分开算，最后加到一起，起始算的还是一个大矩阵
    //这是多个Hessian矩阵
    vector<Eigen::MatrixXd> hessians(thd_num, Hess);
    //这是多个Jacobian矩阵
    vector<Eigen::VectorXd> jacobians(thd_num, JacT);
    //这是多个残差
    vector<double> resis(thd_num, 0);

    //统计有多少个体素滑窗要计算
    uint gps_size = plvec_voxels.size();
    //todo gps_size = 0?
    // ROS_INFO("gps_size: %d", gps_size);
    // ROS_INFO("thd_num: %d", (uint)thd_num);
    if(gps_size < (uint)thd_num)
    {
      // ROS_INFO("divide_thread inside mark 2");
      //?这里好像有问题，算完Hess还要赋值
      acc_t_evaluate(so3_ps, t_ps, 0, gps_size, Hess, JacT, residual);
      Hess = hessians[0];
      JacT = jacobians[0];
      residual = resis[0];
      return;
    }
    
    vector<thread*> mthreads(thd_num);

    //把所有要参与计算的体素滑窗分为thd_num个部分分别计算 最后加到一起
    double part = 1.0*(gps_size)/thd_num;
    for(int i=0; i<thd_num; i++)
    {
      // ROS_INFO("divide_thread inside mark 3");
      //左闭右开区间
      int np = part*i;
      int nn = part*(i+1);

      mthreads[i] = new thread(&LM_SLWD_VOXEL::acc_t_evaluate, this, ref(so3_ps), ref(t_ps), np, nn, ref(hessians[i]), ref(jacobians[i]), ref(resis[i]));
    }

    for(int i=0; i<thd_num; i++)
    {
      // ROS_INFO("divide_thread inside mark 4");
      //回收线程
      mthreads[i]->join();
      //分开计算的结果累加起来
      Hess += hessians[i];
      JacT += jacobians[i];
      residual += resis[i];
      delete mthreads[i];
    }

  }

  // Calculate residual
  /**
   * @brief 计算当前优化器内所有体素滑窗的残差
   * 
   * @param so3_ps 
   * @param t_ps 
   * @param residual 
   */
  void evaluate_only_residual(vector<SO3> &so3_ps, vector<Eigen::Vector3d> &t_ps, double &residual)
  {
    residual = 0;
    uint gps_size = plvec_voxels.size();
    Eigen::Vector3d vec_tran;

    for(uint a=0; a<gps_size; a++)
    {
      uint k = lam_types[a];
      SIG_VEC_CLASS &sig_vec = sig_vecs[a];
      vector<Eigen::Vector3d> &plvec_voxel = *plvec_voxels[a];
      vector<int> &slwd_num = *slwd_nums[a];
      uint backnum = plvec_voxel.size();
      
      Eigen::Vector3d centor(Eigen::Vector3d::Zero());
      Eigen::Matrix3d covMat(Eigen::Matrix3d::Zero());

      for(uint i=0; i<backnum; i++)
      {
        vec_tran = so3_ps[slwd_num[i]].matrix()*plvec_voxel[i] + t_ps[slwd_num[i]];
        centor += vec_tran;
        covMat += vec_tran * vec_tran.transpose();
      }

      double N_points = backnum + sig_vec.sigma_size;
      centor += sig_vec.sigma_vi;
      covMat += sig_vec.sigma_vTv;

      covMat = covMat - centor*centor.transpose()/N_points;
      covMat = covMat / N_points;

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);
      Eigen::Vector3d eigen_value = saes.eigenvalues();
      
      if(k == 1)
      {
        residual += corn_less*eigen_value[k];
      }
      else
      {
        residual += eigen_value[k];
      }
    }
  }

  // LM process
  /**
   * @brief 滑窗优化的过程
   * 
   */
  void damping_iter()
  {
    my_mutex.lock();
    //正在进行滑窗优化
    map_refine_flag = 1;
    my_mutex.unlock();

    //确保数据是正确的 即所有数据结构的体素数量对应上
    if(plvec_voxels.size()!=slwd_nums.size() || plvec_voxels.size()!=lam_types.size() || plvec_voxels.size()!=sig_vecs.size())
    {
      printf("size is not equal\n");
      exit(0);
    }

    //?u是阻尼因子 v是
    double u = 0.01, v = 2;
    //构建Hessian矩阵 D（《十四讲》中LM法中的D矩阵 系数矩阵）:6M x 6M Hess:6M x 6M
    Eigen::MatrixXd D(jac_leng, jac_leng), Hess(jac_leng, jac_leng);
    //J^T 6M x 1 dxi=delta_T:6M x 1
    Eigen::VectorXd JacT(jac_leng), dxi(jac_leng);

    //对应论文公式14
    Eigen::MatrixXd Hess2(jac_leng, jac_leng);
    Eigen::VectorXd JacT2(jac_leng);

    D.setIdentity();
    //?残差
    double residual1, residual2, q;
    //?是否计算Hessian矩阵的标志
    bool is_calc_hess = true;

    //用于求解优化问题AX=B
    //6M x 6M
    cv::Mat matA(jac_leng, jac_leng, CV_64F, cv::Scalar::all(0));
    //6M x 1
    cv::Mat matB(jac_leng, 1, CV_64F, cv::Scalar::all(0));
    //6M x 1
    cv::Mat matX(jac_leng, 1, CV_64F, cv::Scalar::all(0));

    //迭代优化
    for(int i=0; i<iter_max; i++)
    {
      if(is_calc_hess)
      {
        // ROS_INFO("divide_thread outside mark");
        // calculate Hessian, Jacobian, residual
        divide_thread(so3_poses, t_poses, Hess, JacT, residual1);
      }

      //只保留Hess的对角线元素的矩阵 参考《十四讲》系数矩阵D的取法
      D = Hess.diagonal().asDiagonal();
      Hess2 = Hess + u*D;

    //todo 这里D是空的
    // std::string mat_str = "";
    // for (int m = 0; m < D.rows(); ++m) {
    //     for (int n = 0; n < D.cols(); ++n) {
    //         mat_str += std::to_string(D(m, n)) + " ";
    //     }
    //     mat_str += "\n";
    // }
    // // 输出矩阵的值    
    // ROS_INFO("D:\n%s", mat_str.c_str());

      //填满论文公式14所有已知矩阵
      for(int j=0; j<jac_leng; j++)
      {
        //J^T是列向量 取负号存在matB 论文公式14右端
        matB.at<double>(j, 0) = -JacT(j, 0);
        for(int f=0; f<jac_leng; f++)
        {
          matA.at<double>(j, f) = Hess2(j, f);
        }
      }
      //todo 这里矩阵是空的
      // 输出矩阵的值
      // ROS_INFO("matA:");
      // for(int m =0; m < matA.rows; ++m) {
      //   for(int n =0; n < matA.cols; ++n) {
      //     std::cout << matA.at<double>(m, n) <<" ";
      //   }
      //   std::cout << std::endl;
      // }
      // ROS_INFO("matB:");
      // for(int m =0; m < matB.rows; ++m) {
      //   std::cout << matB.at<double>(m) << std::endl;
      // }
      // ROS_INFO("matX:");
      // for(int m =0; m < matX.rows; ++m) {
      //   std::cout << matX.at<double>(m) << std::endl;
      // }
      //求解线性方程组
      cv::solve(matA, matB, matX, cv::DECOMP_QR);
      for(int j=0; j<jac_leng; j++)
      {
        //取出优化结果
        dxi(j, 0) = matX.at<double>(j, 0);
        //todo dxi值为-nan

      }

      for(int j=0; j<slwd_size; j++)
      {
        //todo 这里运行到了 但是上面的矩阵是空的
        // ROS_INFO("mark2 %d", j);
        // // 假设有一个滑窗大小 slwd_size
        // int slwd_size1 = so3_poses.size();

        // // 将 dxi 转换为字符串输出
        // std::string dxi_str = "";
        // for (int i = 0; i < dxi.size(); ++i) {
        //     dxi_str += std::to_string(dxi(i)) + " ";
        // }

        // // 输出 dxi 的值
        // ROS_INFO("dxi: %s", dxi_str.c_str());

        // // 输出 so3_poses 中每个 SO3 对象的值
        // for (int j = 0; j < slwd_size1; ++j) {
        //     Eigen::Quaterniond quaternion = so3_poses[j].unit_quaternion();

        //     // 将 Quaterniond 转换为字符串
        //     std::string quaternion_str = "[" + std::to_string(quaternion.w()) + ", " +
        //                                   std::to_string(quaternion.x()) + ", " +
        //                                   std::to_string(quaternion.y()) + ", " +
        //                                   std::to_string(quaternion.z()) + "]";

        //     // 输出每个 SO3 对象的值
        //     ROS_INFO("so3_poses[%d]: %s", j, quaternion_str.c_str());
        // }

        // left multiplication
        //更新滑窗中每个关键帧的位姿 临时的
        //更新旋转是左乘 因为前面算点坐标对位姿求导用的就是左乘扰动模型
        so3_poses_temp[j] = SO3::exp(dxi.block<3, 1>(6*(j), 0)) * so3_poses[j];
        //更新位移
        t_poses_temp[j] = t_poses[j] + dxi.block<3, 1>(6*(j)+3, 0);
      }
      // ROS_INFO("mark3");

      // LM
      //?这里具体是什么原理 和十四讲对不上
      double q1 = 0.5*(dxi.transpose() * (u*D*dxi-JacT))[0];
      // double q1 = 0.5*dxi.dot(u*D*dxi-JacT);
      //对临时的再做一次处理 决定是否要接受优化结果 这是LM法
      evaluate_only_residual(so3_poses_temp, t_poses_temp, residual2);

      q = (residual1-residual2);
      // printf("residual%d: %lf u: %lf v: %lf q: %lf %lf %lf\n", i, residual1, u, v, q/q1, q1, q);

      if(q > 0)
      {
        so3_poses = so3_poses_temp;
        t_poses = t_poses_temp;
        q = q / q1;
        v = 2;
        q = 1 - pow(2*q-1, 3);
        u *= (q<one_three ? one_three:q);
        is_calc_hess = true;
      }
      else
      {
        u = u * v;
        v = 2 * v;
        is_calc_hess = false;
      }
      
      //如果已经收敛 结束优化
      if(fabs(residual1-residual2)<1e-9)
      {  
        break;
      }
    }
    
    my_mutex.lock();
    //滑窗优化结束
    map_refine_flag = 2;
    my_mutex.unlock();
  }

  int read_refine_state()
  {
    int tem_flag;
    my_mutex.lock();
    tem_flag = map_refine_flag;
    my_mutex.unlock();
    return tem_flag;
  }

  void set_refine_state(int tem)
  {
    my_mutex.lock();
    map_refine_flag = tem;
    my_mutex.unlock();
  }

  /**
   * @brief 清空优化器内所有体素的信息
   * 
   */
  void free_voxel()
  {
    uint a_size = plvec_voxels.size();
    for(uint i=0; i<a_size; i++)
    {
      delete (plvec_voxels[i]);
      delete (slwd_nums[i]);
    }

    plvec_voxels.clear();
    slwd_nums.clear();
    sig_vecs.clear();
    lam_types.clear();
  } 

};


//八叉树节点数据结构
//八叉树是一个嵌套的结构，不考虑再次分割的情况下就对应一个体素
class OCTO_TREE
{
public:
  //静态成员变量，体素滑窗大小
  static int voxel_windowsize;
  //?存储点云指针 存储一帧点云中体素点云中心点在当前帧坐标系下的坐标 每个元素对应滑窗中一帧点云
  //PL_VEC是以vector的形式存储的点云 这里存储当前体素的滑窗内每一帧原始点云
  vector<PL_VEC*> plvec_orig;
  //?存储点云指针 存储一帧点云中体素点云中心点在世界坐标系下的坐标 每个元素对应滑窗中一帧点云
  //存储体素的滑窗内每一帧点云转换到世界坐标系下的坐标
  vector<PL_VEC*> plvec_tran;
  //0代表当前节点是叶子结点，1代表不是叶子节点 初始化为0
  int octo_state; // 0 is end of tree, 1 is not
  //滑窗外的所有点 滑窗外的点就是边缘化掉的点
  PL_VEC sig_vec_points;
  //边缘化掉的点的信息
  SIG_VEC_CLASS sig_vec;
  //代表该体素拟合哪种特征 0代表拟合平面特征 1代表拟合直线特征
  int ftype;
  //统计当前体素滑窗中所有关键帧的点数量
  int points_size, sw_points_size;
  //如果拟合面 这个值=滑窗中所有点构造的协方差矩阵的最大特征值除以最小特征值 如果拟合线 这个值=滑窗中所有中心点构造的协方差矩阵的最大特征值除以第二大特征值
  double feat_eigen_ratio, feat_eigen_ratio_test;
  //存储该体素（八叉树节点）的拟合特征的参数 面：均值点+面的法向量 线：均值点+线的方向向量
  //当前体素拟合特征的参数信息
  PointTypeXYZIN ap_centor_direct;
  //一个八叉树节点是一个体素，这个是当前体素的中心点在世界系下的坐标，例如体素是1m边长，这个就一定是.5结尾，注意不是点云求均值，就是体素中心坐标
  double voxel_center[3]; // x, y, z
  //体素的四分之一边长 用于计算八叉树子节点的体素中心坐标
  double quater_length;
  //存储八个子节点指针
  OCTO_TREE* leaves[8];
  //待拟合标志 如果为1，则用recut递归拟合特征 如果为0说明之前拟合成功，不需要再拟合特征 体素中添加了新点或者第一次构造体素八叉树时会初始化为1
  bool is2opt;
  //体素滑窗的大小
  int capacity;
  //存储当前拟合成功的特征参数 每个元素对应一次成功拟合的参数
  pcl::PointCloud<PointTypeXYZIN> root_centors;

  //构造函数
  OCTO_TREE(int ft, int capa): ftype(ft), capacity(capa)
  {
    //初始化节点状态
    octo_state = 0;
    //初始化子节点指针为空
    for(int i=0; i<8; i++)
    {
      leaves[i] = nullptr;
    }

    for(int i=0; i<capacity; i++)
    {
      //堆区创建点云空指针
      //存储体素滑窗内每一帧原始点云
      plvec_orig.push_back(new PL_VEC());
      //存储体素滑窗内每一帧点云转换到世界坐标系下的坐标
      plvec_tran.push_back(new PL_VEC());
    }
    //构造函数中初始化为1，待拟合
    is2opt = true;
  }

  // Used by "recut"
  /**
   * @brief 计算当前体素内所有点构成的特征参数 面：均值点+面的法向量 线：均值点+线的方向向量
   * 
   */
  void calc_eigen()
  {
    //存储体素内所有点构成的协方差矩阵
    Eigen::Matrix3d covMat(Eigen::Matrix3d::Zero());
    //存储体素内所有点的均值
    Eigen::Vector3d center(0, 0, 0);

    //统计滑窗中第i帧中点的数量
    uint asize;
    //遍历滑窗中关键帧
    for(int i=0; i<OCTO_TREE::voxel_windowsize; i++)
    {
      //用世界坐标系下的坐标
      //滑窗中第i帧中点数量
      asize = plvec_tran[i]->size();
      //遍历当前帧每个点
      //其实就是把当前体素的滑窗内所有的点都算进来
      for(uint j=0; j<asize; j++)
      {
        covMat += (*plvec_tran[i])[j] * (*plvec_tran[i])[j].transpose();
        center += (*plvec_tran[i])[j];
      }
    }

    //v代表边缘化掉的每一个点在世界坐标系下的坐标 sigma_vTv是所有vTv求和
    covMat += sig_vec.sigma_vTv;
    center += sig_vec.sigma_vi;
    //计算均值
    center /= points_size;
    //计算协方差矩阵
    covMat = covMat/points_size - center*center.transpose();
    
    //这个Eigen类用于求解自伴随矩阵的特征值和特征向量
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);
    //ftype 0代表面 1代表线 如果是面 用最大特征值除以最小特征值 如果是线 用最大特征值除以第二大特征值
    feat_eigen_ratio = saes.eigenvalues()[2] / saes.eigenvalues()[ftype];
    //如果拟合面 取最小特征值对应的特征向量 也就是面的法向量 如果拟合线，取最大特征值对应的特征向量 也就是直线的方向向量
    Eigen::Vector3d direct_vec = saes.eigenvectors().col(2*ftype);

    //存储体素内滑窗中所有点的均值 在世界坐标系下
    ap_centor_direct.x = center.x();
    ap_centor_direct.y = center.y();
    ap_centor_direct.z = center.z();
    //存储滑窗中所有中心点构造的这个体素的平面的法向量 或者直线的方向向量
    ap_centor_direct.normal_x = direct_vec.x();
    ap_centor_direct.normal_y = direct_vec.y();
    ap_centor_direct.normal_z = direct_vec.z();
  }

  // Cut root voxel into small pieces
  // frame_head: Position of newest scan in sliding window
  /**
   * @brief 把root voxel递归向下继续切分
   * 
   * @param layer 当前八叉树节点是第几层，从0开始数
   * @param frame_head 滑窗中最新关键帧的索引 例如处理第一个关键帧时这个值为0，处理第二个关键帧时这个值为1
   * @param pl_feat_map 存储当前体素内拟合的特征参数，每个元素都包含均值点+面的法向量/线的方向向量的信息
   *                    这个参数传递引用，每次递归都继续向下传递，成功拟合特征，就会把特征参数存入这个结构中
   */
  void recut(int layer, uint frame_head, pcl::PointCloud<PointTypeXYZIN> &pl_feat_map)
  {
    //0代表是叶子结点 初始化为0 调用这个函数是待拟合的体素，octo_state初始化为0与待拟合不冲突，例如新建立的体素，添加了新点但是之前也成功拟合特征的体素
    //如果这个标志为1，代表之前调用recut时没有成功拟合，那么这一次就不用再计算特征值，直接进行下面的步骤，也就是八叉树向下分裂
    if(octo_state == 0)
    {
      //初始化当前体素内点云点数为0
      points_size = 0;
      //遍历体素内滑窗
      for(int i=0; i<OCTO_TREE::voxel_windowsize; i++)
      {
        //滑窗中第i帧点云内中心点的数量累加 统计体素滑窗中所有点的数量
        points_size += plvec_orig[i]->size();
      }
      
      //算上边缘化掉的点数量
      points_size += sig_vec.sigma_size;
      //如果中心点数量太少 返回 拟合特征失败
      if(points_size < MIN_PS)
      {
        //这里初始化为-1，代表无意义；如果是拟合面 用最大特征值除以最小特征值 如果是拟合线 用最大特征值除以第二大特征值
        feat_eigen_ratio = -1;
        return;
      }

      //计算当前体素内的拟合特征参数 面：均值点+面的法向量 线：均值点+线的方向向量
      //拟合特征
      calc_eigen(); // calculate eigenvalue ratio
      
      //如果出现nan值，特征拟合失败，直接返回 失败返回不递归，因为已经出现了nan值，没必要再递归拟合
      if(isnan(feat_eigen_ratio))
      {
        feat_eigen_ratio = -1;
        return;
      }

      //判断特征拟合是否成功 如果成功把特征参数存入pl_feat_map 每个元素对应一次成功拟合的参数，例如假设这是第二次拟合，成功了，
      //那就把第二次的参数存进去，第一次的也留着呢 成功返回，不需要递归拟合
      if(feat_eigen_ratio >= feat_eigen_limit[ftype])
      {
        pl_feat_map.push_back(ap_centor_direct);
        return;
      }

      // if(layer == 3)
      //这是一个递归函数，如果已经尝试拟合了前四层（0,1,2,3），还没有成功，那么直接返回，不再递归拟合
      if(layer == 4)   
      {
        return;
      }
      //到了这里，既没有成功也没有完全失败，需要向下分裂八叉树继续尝试拟合，把标志置为1，即非八叉树叶子节点，还需要尝试拟合
      octo_state = 1;
      // All points in slidingwindow should be put into subvoxel
      //这里置为0和下面遍历次数相关
      frame_head = 0; 
    }

    //走到这里都是当前节点本次拟合不成功，而且没有放弃，还需要向下分裂八叉树

    //记录子节点索引，用于分裂八叉树
    int leafnum;
    //用于遍历
    uint a_size;

    //从体素滑窗内的第一帧开始遍历滑窗
    for(int i=frame_head; i<OCTO_TREE::voxel_windowsize; i++)
    {
      //遍历体素滑窗内当前关键帧的所有点
      a_size = plvec_tran[i]->size();

      //计算当前中心点应该属于八叉树的哪个子节点
      for(uint j=0; j<a_size; j++)
      {
        int xyz[3] = {0, 0, 0};
        for(uint k=0; k<3; k++)
        {
          if((*plvec_tran[i])[j][k] > voxel_center[k])
          {
            xyz[k] = 1;
          }
        }
        leafnum = 4*xyz[0] + 2*xyz[1] + xyz[2];
        //如果该子节点还不存在，就新建一个子节点
        if(leaves[leafnum] == nullptr)
        {
          //新建的子节点和当前节点拟合相同特征 容纳相同数量帧的点云
          leaves[leafnum] = new OCTO_TREE(ftype, capacity);
          //计算新的子节点的体素中心在世界系下的坐标
          leaves[leafnum]->voxel_center[0] = voxel_center[0] + (2*xyz[0]-1)*quater_length;
          leaves[leafnum]->voxel_center[1] = voxel_center[1] + (2*xyz[1]-1)*quater_length;
          leaves[leafnum]->voxel_center[2] = voxel_center[2] + (2*xyz[2]-1)*quater_length;
          //计算新的子节点的体素的四分之一长度
          leaves[leafnum]->quater_length = quater_length / 2;
        }
        //子节点体素内的每个点在LiDAR系下的坐标和在世界系下的坐标也要传给子节点
        leaves[leafnum]->plvec_orig[i]->push_back((*plvec_orig[i])[j]);
        leaves[leafnum]->plvec_tran[i]->push_back((*plvec_tran[i])[j]);
      }
    }
    
    //如果不是八叉树的第一层
    if(layer != 0)
    {
      //遍历滑窗中所有关键帧
      for(int i=frame_head; i<OCTO_TREE::voxel_windowsize; i++)
      {
        //当前关键帧中中心点数量不为0
        if(plvec_orig[i]->size() != 0)
        {
          //清空当前八叉树节点的点云，因为走到这里已经把当前节点的中心点传给了子节点，所以都清空
          //第一层八叉树的数据要留下
          vector<Eigen::Vector3d>().swap(*plvec_orig[i]);
          vector<Eigen::Vector3d>().swap(*plvec_tran[i]);
        }
      }
    }

    //层数+1
    layer++;
    //遍历当前节点的八个子节点
    for(uint i=0; i<8; i++)
    {
      //如果有子节点
      if(leaves[i] != nullptr)
      {
        //递归调用，继续尝试拟合
        leaves[i]->recut(layer, frame_head, pl_feat_map);
      }
    }

  }

  // marginalize 5 scans in slidingwindow (assume margi_size is 5)
  /**
   * @brief 边缘化函数
   * 
   * @param layer 八叉树地图层数
   * @param margi_size 边缘化掉几帧
   * @param q_poses 滑窗中关键帧的旋转矩阵
   * @param t_poses 滑窗中关键帧的位移
   * @param window_base 滑窗起始帧索引
   * @param pl_feat_map 存储拟合出的特征参数
   */
  void marginalize(int layer, int margi_size, vector<Eigen::Quaterniond> &q_poses, vector<Eigen::Vector3d> &t_poses, int window_base, pcl::PointCloud<PointTypeXYZIN> &pl_feat_map)
  {
    //如果是叶子节点或者八叉树地图第一层 只对已经成功拟合特征的体素进行边缘化，对第一层八叉树也进行边缘化
    if(octo_state!=1 || layer==0)
    {
      if(octo_state != 1)
      {
        for(int i=0; i<OCTO_TREE::voxel_windowsize; i++)
        {
          // Update points by new poses
          //更新体素内所有点位姿
          //?为什么要加window_base
          plvec_trans_func(*plvec_orig[i], *plvec_tran[i], q_poses[i+window_base].matrix(), t_poses[i+window_base]);
        }
      }

      // Push front 5 scans into P_fix
      uint a_size;
      //特征拟合成功
      if(feat_eigen_ratio > feat_eigen_limit[ftype])
      {
        //把要边缘化的关键帧的点全部放到sig_vec_points中
        for(int i=0; i<margi_size; i++)
        {
          sig_vec_points.insert(sig_vec_points.end(), plvec_tran[i]->begin(), plvec_tran[i]->end());
        }
        //对所有边缘化掉的点降采样一次
        down_sampling_voxel(sig_vec_points, quater_length);

        //要边缘化掉的点的数量 
        a_size = sig_vec_points.size();
        //先清空边缘化点信息
        sig_vec.tozero();
        //更新要边缘化掉的点的数量
        sig_vec.sigma_size = a_size;
        //更新边缘化点的信息
        for(uint i=0; i<a_size; i++)
        {
          sig_vec.sigma_vTv += sig_vec_points[i] * sig_vec_points[i].transpose();
          sig_vec.sigma_vi  += sig_vec_points[i];
        }
      }

      // Clear front 5 scans
      //清空边缘化掉的点云
      for(int i=0; i<margi_size; i++)
      {
        PL_VEC().swap(*plvec_orig[i]);
        PL_VEC().swap(*plvec_tran[i]);
        // plvec_orig[i].clear(); plvec_orig[i].shrink_to_fit();
      }

      //如果是八叉树第一层，把体素滑窗中剩下的点统计一下，如果没有点，修改标志
      if(layer == 0)
      {
        a_size = 0;
        for(int i=margi_size; i<OCTO_TREE::voxel_windowsize; i++)
        {
          a_size += plvec_orig[i]->size();
        }
        if(a_size == 0)
        {
          // Voxel has no points in slidingwindow
          is2opt = false;
        }
      }
      
      //体素滑窗中剩下的点向前移动
      for(int i=margi_size; i<OCTO_TREE::voxel_windowsize; i++)
      {
        plvec_orig[i]->swap(*plvec_orig[i-margi_size]);
        plvec_tran[i]->swap(*plvec_tran[i-margi_size]);
      }
      
      //如果当前八叉树节点是叶子节点
      if(octo_state != 1)
      {
        //边缘化以后更新体素滑窗中点的数量 要算上边缘化掉的点的数量
        points_size = 0;
        for(int i=0; i<OCTO_TREE::voxel_windowsize-margi_size; i++)
        {
          points_size += plvec_orig[i]->size();
        }
        points_size += sig_vec.sigma_size;
        //点太少就拟合失败
        if(points_size < MIN_PS)
        {
          feat_eigen_ratio = -1;
          return;
        }

        //边缘化以后重新计算特征值 拟合特征
        calc_eigen();

        //拟合失败
        if(isnan(feat_eigen_ratio))
        {
          feat_eigen_ratio = -1;
          return;
        }
        //拟合成功
        if(feat_eigen_ratio >= feat_eigen_limit[ftype])
        {
          pl_feat_map.push_back(ap_centor_direct);
        }
      }
    }
    
    //如果不是叶子节点，向更深层递归进行边缘化 因为只对已经成功拟合特征的体素进行边缘化
    if(octo_state == 1)
    {
      layer++;
      for(int i=0; i<8; i++)
      {
        if(leaves[i] != nullptr)
        {
          leaves[i]->marginalize(layer, margi_size, q_poses, t_poses, window_base, pl_feat_map);
        }
      }
    }
  }

  // Used by "traversal_opt"
  /**
   * @brief 计算这个体素的滑窗内的点构成的点云的协方差矩阵，进而计算协方差矩阵的特征值之间的比例，用于判断特征拟合是否成功
   * 
   */
  void traversal_opt_calc_eigen()
  {
    Eigen::Matrix3d covMat(Eigen::Matrix3d::Zero());
    Eigen::Vector3d center(0, 0, 0);
   
    uint asize;
    //遍历滑窗中每个关键帧
    for(int i=0; i<OCTO_TREE::voxel_windowsize; i++)
    {
      //统计当前关键帧中中心点的数量
      asize = plvec_tran[i]->size();
      //遍历每个中心点
      for(uint j=0; j<asize; j++)
      {
        covMat += (*plvec_tran[i])[j] * (*plvec_tran[i])[j].transpose();
        center += (*plvec_tran[i])[j];
      }
    }

    //计算当前体素点云的协方差矩阵
    covMat -= center*center.transpose()/sw_points_size; 
    covMat /= sw_points_size;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);
    //如果拟合面 这个值=滑窗中所有中心点构造的协方差矩阵的最大特征值除以最小特征值 
    //如果拟合线 这个值=滑窗中所有中心点构造的协方差矩阵的最大特征值除以第二大特征值
    feat_eigen_ratio_test = (saes.eigenvalues()[2] / saes.eigenvalues()[ftype]);
  }

  // Push voxel into "opt_lsv" (LM optimizer)
  /**
   * @brief 把体素（八叉树）信息放入优化器
   * 
   * @param opt_lsv LM优化器
   */
  void traversal_opt(LM_SLWD_VOXEL &opt_lsv)
  {
    //如果是叶子节点 要放入滑窗优化器
    if(octo_state != 1)
    {
      //初始化为0 统计当前体素滑窗中所有帧的中心点数量
      sw_points_size = 0;
      //遍历当前体素的滑窗中每个关键帧
      for(int i=0; i<OCTO_TREE::voxel_windowsize; i++)
      {
        //统计体素的滑窗内所有点数量
        sw_points_size += plvec_orig[i]->size();
      }
      //如果中心点数量太少，直接返回
      if(sw_points_size < MIN_PS)
      {
        return;
      }
      //计算这个体素的滑窗内的点构成的点云的协方差矩阵，进而计算协方差矩阵的特征值之间的比例，用于判断特征拟合是否成功
      traversal_opt_calc_eigen();

      //如果是nan值，直接返回
      if(isnan(feat_eigen_ratio_test))
      {
        return;
      }

      //如果特征拟合成功
      if(feat_eigen_ratio_test > opt_feat_eigen_limit[ftype])
      {
        //把当前体素的滑窗中的所有关键帧的点云和当前的边缘化残留的信息传递给滑窗优化器
        // ROS_INFO("push_voxel outside mark");
        opt_lsv.push_voxel(plvec_orig, sig_vec, ftype);
      }

    }
    //如果不是叶子节点
    else
    {
      //遍历当前节点的子节点
      for(int i=0; i<8; i++)
      {
        //如果子节点存在，递归调用traversal_opt，尝试把体素（八叉树）信息放入优化器
        if(leaves[i] != nullptr)
        {
          leaves[i]->traversal_opt(opt_lsv);
        }
      }
    }
  }

};

//静态成员变量初始化
int OCTO_TREE::voxel_windowsize = 0;

// Like "LM_SLWD_VOXEL"
// Scam2map optimizer
class VOXEL_DISTANCE
{
public:
  SO3 so3_pose, so3_temp;
  Eigen::Vector3d t_pose, t_temp;
  PL_VEC surf_centor, surf_direct;
  PL_VEC corn_centor, corn_direct;
  PL_VEC surf_gather, corn_gather;
  vector<double> surf_coeffs, corn_coeffs;

  void push_surf(Eigen::Vector3d &orip, Eigen::Vector3d &centor, Eigen::Vector3d &direct, double coeff)
  {
    direct.normalize();
    surf_direct.push_back(direct); surf_centor.push_back(centor);
    surf_gather.push_back(orip); surf_coeffs.push_back(coeff);
  }

  void push_line(Eigen::Vector3d &orip, Eigen::Vector3d &centor, Eigen::Vector3d &direct, double coeff)
  {
    direct.normalize();
    corn_direct.push_back(direct); corn_centor.push_back(centor);
    corn_gather.push_back(orip); corn_coeffs.push_back(coeff);
  }

  void evaluate_para(SO3 &so3_p, Eigen::Vector3d &t_p, Eigen::Matrix<double, 6, 6> &Hess, Eigen::Matrix<double, 6, 1> &g, double &residual)
  {
    Hess.setZero(); g.setZero(); residual = 0;
    uint a_size = surf_gather.size();
    for(uint i=0; i<a_size; i++)
    {
      Eigen::Matrix3d _jac = surf_direct[i] * surf_direct[i].transpose();
      Eigen::Vector3d vec_tran = so3_p.matrix() * surf_gather[i];
      Eigen::Matrix3d point_xi = -SO3::hat(vec_tran);
      vec_tran += t_p;

      Eigen::Vector3d v_ac = vec_tran - surf_centor[i];
      Eigen::Vector3d d_vec = _jac * v_ac;
      Eigen::Matrix<double, 3, 6> jacob;
      jacob.block<3, 3>(0, 0) = _jac * point_xi;
      jacob.block<3, 3>(0, 3) = _jac;

      residual += surf_coeffs[i] * d_vec.dot(d_vec);
      Hess += surf_coeffs[i] * jacob.transpose() * jacob;
      g += surf_coeffs[i] * jacob.transpose() * d_vec;
    }

    a_size = corn_gather.size();
    for(uint i=0; i<a_size; i++)
    {
      Eigen::Matrix3d _jac = Eigen::Matrix3d::Identity() - corn_direct[i] * corn_direct[i].transpose();
      Eigen::Vector3d vec_tran = so3_p.matrix() * corn_gather[i];
      Eigen::Matrix3d point_xi = -SO3::hat(vec_tran);
      vec_tran += t_p;

      Eigen::Vector3d v_ac = vec_tran - corn_centor[i];
      Eigen::Vector3d d_vec = _jac * v_ac;
      Eigen::Matrix<double, 3, 6> jacob;
      jacob.block<3, 3>(0, 0) = _jac * point_xi;
      jacob.block<3, 3>(0, 3) = _jac;

      residual += corn_coeffs[i] * d_vec.dot(d_vec);
      Hess += corn_coeffs[i] * jacob.transpose() * jacob;
      g += corn_coeffs[i] * jacob.transpose() * d_vec;
    }
  }

  void evaluate_only_residual(SO3 &so3_p, Eigen::Vector3d &t_p, double &residual)
  {
    residual = 0;
    uint a_size = surf_gather.size();
    for(uint i=0; i<a_size; i++)
    {
      Eigen::Matrix3d _jac = surf_direct[i] * surf_direct[i].transpose();
      Eigen::Vector3d vec_tran = so3_p.matrix() * surf_gather[i];
      vec_tran += t_p;

      Eigen::Vector3d v_ac = vec_tran - surf_centor[i];
      Eigen::Vector3d d_vec = _jac * v_ac;

      residual += surf_coeffs[i] * d_vec.dot(d_vec);
    }

    a_size = corn_gather.size();
    for(uint i=0; i<a_size; i++)
    {
      Eigen::Matrix3d _jac = Eigen::Matrix3d::Identity() - corn_direct[i] * corn_direct[i].transpose();
      Eigen::Vector3d vec_tran = so3_p.matrix() * corn_gather[i];
      vec_tran += t_p;

      Eigen::Vector3d v_ac = vec_tran - corn_centor[i];
      Eigen::Vector3d d_vec = _jac * v_ac;

      residual += corn_coeffs[i] * d_vec.dot(d_vec);
    }

  }

  void damping_iter()
  {
    double u = 0.01, v = 2;
    Eigen::Matrix<double, 6, 6> D; D.setIdentity();
    Eigen::Matrix<double, 6, 6> Hess, Hess2;
    Eigen::Matrix<double, 6, 1> g;
    Eigen::Matrix<double, 6, 1> dxi;
    double residual1, residual2;

    cv::Mat matA(6, 6, CV_64F, cv::Scalar::all(0));
    cv::Mat matB(6, 1, CV_64F, cv::Scalar::all(0));
    cv::Mat matX(6, 1, CV_64F, cv::Scalar::all(0));

    for(int i=0; i<20; i++)
    {
      evaluate_para(so3_pose, t_pose, Hess, g, residual1);
      D = Hess.diagonal().asDiagonal();
      
      // dxi = (Hess + u*D).bdcSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(-g);

      Hess2 = Hess + u*D;
      for(int j=0; j<6; j++)
      {
        matB.at<double>(j, 0) = -g(j, 0);
        for(int f=0; f<6; f++)
        {
          matA.at<double>(j, f) = Hess2(j, f);
        }
      }
      cv::solve(matA, matB, matX, cv::DECOMP_QR);
      for(int j=0; j<6; j++)
      {
        dxi(j, 0) = matX.at<double>(j, 0);
      }

      so3_temp = SO3::exp(dxi.block<3, 1>(0, 0)) * so3_pose;
      t_temp = t_pose + dxi.block<3, 1>(3, 0);
      evaluate_only_residual(so3_temp, t_temp, residual2);
      double q1 = dxi.dot(u*D*dxi-g);
      double q = residual1 - residual2;
      // printf("residual: %lf u: %lf v: %lf q: %lf %lf %lf\n", residual1, u, v, q/q1, q1, q);
      if(q > 0)
      {
        so3_pose = so3_temp;
        t_pose = t_temp;
        q = q / q1;
        v = 2;
        q = 1 - pow(2*q-1, 3);
        u *= (q<one_three ? one_three:q);
      }
      else
      {
        u = u * v;
        v = 2 * v;
      }

      if(fabs(residual1-residual2)<1e-9)
      {
        break;
      }

    }

  }

};


#endif

