# 编写简单的消息发布器和订阅器 
## 编写发布器节点

```c++
#include "ros/ros.h"
//存放在 std_msgs package 里，是由 String.msg 文件自动生成的头文件
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv)
{
  /**
   * 初始化 ROS 
   * 可以指定节点的名称——运行过程中，节点的名称必须唯一
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * 告诉 master 我们将要在 chatter（话题名） 上发布 std_msgs/String 消息类型的消息
   * 第二个参数是发布序列的大小。如果我们发布的消息的频率太高，缓冲区中的消息在大于 1000 个的时候就会开始丢弃先前发布的消息。
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  //ros::Rate 对象可以允许你指定自循环的频率
  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /**
     *向所有订阅 chatter 话题的节点发送消息
     */
    chatter_pub.publish(msg);

    /**
     * 用于调用回调函数
     */
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  return 0;
}
```
## 编写订阅器节点
```c++
#include "ros/ros.h"
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  /**
   * 告诉 master 我们要订阅 chatter 话题上的消息。
   * 当有消息发布到这个话题时，ROS 就会调用 chatterCallback() 函数
   * 第二个参数是队列大小
   */
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
```
## buid
在 CMakeLists.txt 文件末尾加入几条语句:
```cmake
## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_generate_messages_cpp)
```
## Pyhton

