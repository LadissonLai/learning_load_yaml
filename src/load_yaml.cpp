#include <ros/ros.h>

#include <iostream>

int main(int argc, char** argv) {
  ros::init(argc, argv, "load_yaml");
  ros::NodeHandle nh;

  std::string node_name = ros::this_node::getName();
  std::cout << "-------------------------------" << std::endl;
  std::cout << "Node name(include ns): " << node_name << std::endl;

  std::string global_car_name;
  std::string inner_car_name;
  // 获取全局yaml参数
  nh.getParam("/car_name", global_car_name);
  std::cout << "global_car_name: " << global_car_name << std::endl;
  // 获取node内部参数
  nh.getParam(node_name + "/car_name", inner_car_name);
  std::cout << "inner_car_name: " << inner_car_name << std::endl;
  // 获取double类型
  double car_width;
  nh.getParam(node_name + "/DWAParams/car_width", car_width);
  std::cout << "car_width: " << car_width << std::endl;
  // 获取bool类型
  bool isRecover;
  nh.getParam(node_name + "/DWAParams/isRecover", isRecover);
  std::cout << "isRecover: " << isRecover << std::endl;
  // 获取数组
  std::vector<double> origin_pose;
  nh.getParam(node_name + "/DWAParams/origin_pose", origin_pose);
  std::cout << "origin_pose: ";
  for (auto i : origin_pose) {
    std::cout << i << " ";
  }
  std::cout << std::endl;
  // 获取嵌套的yaml
  std::string scan_topic_name;
  nh.getParam(node_name + "/DWAParams/scan/scan_topic", scan_topic_name);
  std::cout << "scan_topic_name: " << scan_topic_name << std::endl;
  ros::spin();
  return 0;
}