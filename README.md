# ros加载yaml文件

ROS官方提供了自动加载yaml文件的功能，并且集成到了launch文件里面，只需要使用rosparam标签就能把yaml配置文件加载到ros的参数服务器里面，然后使用nodehandle.getParam()函数就可以方便的使用了。

下面给出关键的使用步骤：
1. 编写yaml文件。注: 避免使用-分割数组，否则容易产生bug，数组使用[]，逗号分隔元素。
2. 编写launch文件，加载yaml。核心命令<rosparam command="load" file="filepath" />
3. cpp文件中访问yaml。直接使用nodehandle.getParam方法，注意参数的命名空间即可。

## launch文件

```xml
<launch>
	<arg name="global_file" default="global_param.yaml" />
	<!-- 在node节点外部添加yaml文件，加载到参数服务器 -->
	<rosparam command="load" file="$(find learning_load_yaml)/param/$(arg global_file)" />
</launch>
```

直接使用<rosparam command="load" file="file_path" />就能将yaml文件解析成内存cpp对象，然后调用参数服务器访问。

## 代码中访问

```cpp
int main(int argc, char** argv) {
  ros::init(argc, argv, "load_yaml");
  ros::NodeHandle nh;
  std::string global_car_name;
  // 获取全局yaml参数
  nh.getParam("/car_name", global_car_name);
  std::cout << "global_car_name: " << global_car_name << std::endl;
}
```

备注：yaml中的数组对应cpp标准库中的std::vector，字符串对应为std::string

## 功能包使用方法

```shell
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone ...
cd ~/catkin_ws
catkin_make
source ./devel/setup.bash
roslaunch learning_load_yaml start.launch
```
## 完整项目
完整项目请查看github[仓库](https://github.com/LadissonLai/learning_load_yaml)。

