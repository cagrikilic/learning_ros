Before creating new ROS code, one must establish a directory (a ROS workspace) where ROS code will reside.

Setting up a ROS workspace (and automating defining ROS paths) needs to be done only once. The process is described at: http://wiki.ros.org/ROS/Tutorials/
InstallingandConfiguringROSEnvironment.

``
$ mkdir -p ~/NAME_ws/src
$ cd ~/NAME_ws/
$ catkin_make
``

The first thing to do when starting to design new ROS code is to create a package. Inside of the /src create a package named "my_minimal_nodes" with dependencies "roscpp" and "std_msgs"

``
catkin_create_pkg my_minimal_nodes roscpp std_msgs
``

By convention, package names should follow common C variable naming conventions: lower case, start with a letter, use underscore sep-
arators, e.g. grasp_planner.

After configuring (if necessary) dependencies in the package.xml, open an editor and create a file called minimal_publisher.cpp and enter the code below

``

``

Much of this code is ROS-specific and may seem cryptic. However, most of the lines are common boilerplate, and
becoming familiar with these common lines will make other ROS code easier to read.

### Important Notes

``
ros::Publisher my_publisher_object = n.advertise<std_msgs::Float64>("topic1", 1)
``

This creates an object to be called my_publisher_object to inform ROS system that the node 'minimal_publisher' intends to publish messages of type std_msgs::Float64 on a topic named topic1.

``
std_msgs::Float64 input_float
``

This creates an object of type std_msgs::Float64 and calls it input_float. The object is defined as having a member called "data"

``
input_float.data =0.0
``

This initializes the "data" member of input_float to the value 0.0. That "data" would be different if the message type was different.

### Compiling ROS nodes

ROS nodes are compiled by running catkin_make. This command must be executed from a specific directory. In a terminal, navigate to your ROS workspace ( ~/ros_ws). Then enter
catkin_make. Before compiling we have to inform catkin_make of the existence of our new source code, minimal_publisher.cpp. To do so, we should edit the CMakeLists.txt 

//Declare a cpp executable
``
add_executable(my_minimal_publishersrc/minimal_publisher.cpp)
``
// Specify libraries to link a library or executable target against
``
target_link_libraries(my_minimal_publisher ${catkin_LIBRARIES})
``

and catkin_make again in ros_ws.

To run the ROS nodes

``
rosrun my_minimal_nodes my_minimal_publisher
``


