# 创建ROS消息和ROS服务
## 消息(msg)和服务(srv)介绍
在ROS中有一个特殊的数据类型：Header，它含有**时间戳和坐标系信息**。在msg文件的第一行经常可以看到Header header的声明.  
## 1. 使用msg
### 1.1 创建一个msg
查看package.xml, 确保它包含一下两条语句:
```xml
<build_depend>message_generation</build_depend>
<run_depend>message_runtime</run_depend>
```
如果没有，添加进去。注意，在构建的时候，我们只需"message_generation"。然而，在运行的时候，我们只需要"message_runtime"。  
在 CMakeLists.txt文件中，利用find_packag函数，增加对message_generation的依赖，可以直接在COMPONENTS的列表里增加message_generation，就像这样：
```cmake
# Do not just add this line to your CMakeLists.txt, modify the existing line
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation)
```
同样，确保你设置了运行依赖：
```cmake
catkin_package(
  ...
  CATKIN_DEPENDS message_runtime ...
  ...)
```
找到如下代码块:
```cmake
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )
```
去掉注释符号#，用你的.msg文件替代Message*.msg，就像下边这样：
```cmake
add_message_files(
  FILES
  Num.msg
)
```
手动添加.msg文件后，我们要确保CMake知道在什么时候重新配置我们的project。 确保添加了如下代码:
```cmake
generate_messages()
```
### 1.2 使用 rosmsg
下面通过rosmsg show命令，检查ROS是否能够识消息
```
$ rosmsg show [message type]
```
eg:
```
$ rosmsg show beginner_tutorials/Num
```
## 2. 使用 srv
### 2.1 创建一个srv
其他的package中复制一个服务。 roscp是一个很实用的命令行工具，它实现了将文件从一个package复制到另外一个package的功能。
```
$ roscp [package_name] [file_to_copy_path] [copy_path]
```
从rospy_tutorials package中复制一个服务文件了：
```
$ roscp rospy_tutorials AddTwoInts.srv srv/AddTwoInts.srv
```
在CMakeLists.txt文件中增加了对message_generation的依赖。:
```cmake
# Do not just add this line to your CMakeLists.txt, modify the existing line
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation)
```
去掉注释，用你自己的srv文件名替换掉那些Service*.srv文件:
```cmake
add_service_files(
  FILES
  AddTwoInts.srv
)
```
### 2.2 使用 rossrv
使用方法:
```shell
$ rossrv show <service type>
```
例子:
```shell
$ rossrv show beginner_tutorials/AddTwoInts
```
## 3. msg和srv都需要的步骤
```cmake
# generate_messages(
#   DEPENDENCIES
# #  std_msgs  # Or other packages containing msgs
# )
```
去掉注释并附加上所有你消息文件所依赖的那些含有.msg文件的package（这个例子是依赖std_msgs,不要添加roscpp,rospy)，结果如下:
```cmake
generate_messages(
  DEPENDENCIES
  std_msgs
)
```
