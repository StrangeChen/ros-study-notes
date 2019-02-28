# 创建ROS程序包
## 创建一个空白的catkin工作空间
首先我们要在计算机上创建一个初始的catkin_ws/路径，这也是catkin工作空间结构的最高层级。输入下列指令，完成初始创建。
```shell
$ mkdir -p ~/catkin_ws/src　　
$ cd ~/catkin_ws/
$ catkin_make #初始化工作空间
```
* src/: ROS的catkin软件包（源代码包）
* build/: catkin（CMake）的缓存信息和中间文件
* devel/: 生成的目标文件（包括头文件，动态链接库，静态链接库，可执行文件等）、环境变量
## 使用catkin_make进行编译
```shell
$ cd ~/catkin_ws #回到工作空间,catkin_make必须在工作空间下执行
$ catkin_make    #开始编译
$ source ~/catkin_ws/devel/setup.bash #刷新坏境
```
如果有新的目标文件产生（原来没有），那么一般紧跟着要source刷新环境，使得系统能够找到刚才编译生成的ROS可执行文件
## 创建一个catkin程序包
创建一个package需要在`catkin_ws/src`下,用到`catkin_create_pkg`命令，用法是：
```shell
catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
```
其中package是包名，depends是依赖的包名，可以依赖多个软件包。
新建一个package叫做test_pkg,依赖roscpp、rospy、std_msgs(常用依赖)
```shell
$ catkin_create_pkg test_pkg roscpp rospy std_msgs
```
## 自定义你的程序包
### 自定义 package.xml
```xml
<pacakge>               根标记文件  
<name>                  包名  
<version>               版本号  
<description>           内容描述  
<maintainer>            维护者 
<license>               软件许可证  
<buildtool_depend>      编译构建工具，通常为catkin    
<depend>                指定依赖项为编译、导出、运行需要的依赖，最常用
<build_depend>          编译依赖项  
<build_export_depend>   导出依赖项
<exec_depend>           运行依赖项
<test_depend>           测试用例依赖项  
<doc_depend>            文档依赖项
```
首先更新描述标签,将描述信息修改为任何你喜欢的内容
```xml
<description>The beginner_tutorials package</description>
```
许可标签:使用BSD协议，因为ROS核心组件的剩余部分已经使用了该协议
```xml
<license>BSD</license>
```
编译和运行时我们需要用到所有指定的依赖包
```xml
 <buildtool_depend>catkin</buildtool_depend>

  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>

  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
```