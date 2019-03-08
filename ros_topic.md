# ROS Topics
## 使用 rqt_graph
```shell
$ rosrun rqt_graph rqt_graph
```

![ros graph](http://wiki.ros.org/cn/ROS/Tutorials/UnderstandingTopics?action=AttachFile&do=get&target=rqt_graph_turtle_key.png)
## rostopic
### rostopic echo
rostopic echo可以显示在某个话题上发布的数据。
```shell
$ rostopic echo [topic]
```
### rostopic list
rostopic list能够列出所有当前订阅和发布的话题。
```
Usage: rostopic list [/topic]

Options:
  -h, --help            show this help message and exit
  -b BAGFILE, --bag=BAGFILE
                        list topics in .bag file
  -v, --verbose         list full details about each topic
  -p                    list only publishers
  -s                    list only subscribers
```
```shell
$ rostopic list -v
```
这会显示出有关所发布和订阅的话题及其类型的详细信息。  
### rostopic type
rostopic type命令用来查看所发布话题的消息类型。
```shell
$ rostopic type [topic]
```
我们可以使用rosmsg命令来查看消息的详细情况：
```shell
$ rosmsg show geometry_msgs/Twist
```
### rostopic pub
rostopic pub可以把数据发布到当前某个正在广播的话题上。
```shell
$ rostopic pub [topic] [msg_type] [args]
```
eg:
```shell
$ rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
```
* -1 这个参数选项使rostopic发布一条消息后马上退出。
* /turtle1/cmd_vel 这是消息所发布到的话题名称
* geometry_msgs/Twist 这是所发布消息的类型
* -- 告诉命令选项解析器接下来的参数部分都不是命令选项
* '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]' 参数
## 使用 rqt_plot
rqt_plot命令可以实时显示一个发布到某个话题上的数据变化图形
```shell
$ rosrun rqt_plot rqt_plot
```
