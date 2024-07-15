/*
 * @Author: cuitongxin cuitongxin201024@163.com
 * @Date: 2024-02-27 18:16:48
 * @LastEditors: cuitongxin cuitongxin201024@163.com
 * @LastEditTime: 2024-02-27 18:17:45
 * @FilePath: /LIO-SAM-DetailedNote/src/path_save.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "utility.h"

void path_save(nav_msgs::Odometry odomAftMapped ){
 
	    //保存轨迹，path_save是文件目录,txt文件提前建好,/home/xxx/xxx.txt,
   			std::ofstream pose1(“path_save”, std::ios::app);
			pose1.setf(std::ios::scientific, std::ios::floatfield);
			pose1.precision(9);
	
			static double timeStart = odomAftMapped.header.stamp.toSec();
			auto T1 =ros::Time().fromSec(timeStart) ;
			pose1<< odomAftMapped.header.stamp -T1<< " "
              << -odomAftMapped.pose.pose.position.y << " "
              << odomAftMapped.pose.pose.position.z << " "
              << odomAftMapped.pose.pose.position.x << " "
              << odomAftMapped.pose.pose.orientation.x << " "
              << odomAftMapped.pose.pose.orientation.y << " "
              << odomAftMapped.pose.pose.orientation.z << " "
              << odomAftMapped.pose.pose.orientation.w << std::endl;
			pose1.close();
            
}
 
int main(int argc, char **argv){
    ros::init(argc, argv, "path_save");
    ros::NodeHandle nh;
    ros::Subscriber save_path = nh.subscribe<nav_msgs::Odometry>("/lio_sam/mapping/odometry_incremental",     100, path_save);	    //保存轨迹，a_loam直接订阅话题/aft_mapped_to_init。
 
    ros::spin();
    }