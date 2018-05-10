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

### Minimal ROS Publisher

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

We have seen that our example publisher is abusive of both CPU capacity and communica-
tions bandwidth. In fact, it would be unusual for a node within a robotic system to require
updates at 30 kHz. A more reasonable update rate for even time-critical, low-level nodes is
1 kHz. In the present example, we will slow our publisher to 1 Hz using a ROS timer.

``
ros::Rate naptime(1.0)
``
OPTIONAL. This creates a ros object from the ros Rate class. Set the sleep timer for 1 Hz repetition rate ( arg is in units of Hz )

``
naptime.sleep()
``
OPTIONAL. This will cause the loop to sleep for the balace of the desired period to achieve the specified loop frequency.


### Minimal ROS Subscriber
Do the same steps as creating a publisher and write the code below;

``

``
#### Important Notes

``
void myCallback(const std_msgs::Float64& message_holder)
``
This function has an argument of a reference pointer (indicated by the & sign) to an object
of type std_msgs::Float64. This is the message type associated with topic1, as published
by our minimal publisher.

The importance of the callback function is that it is awakened when new data is available
on its associated topic (which is set to topic1 in this example). When new data is published
to the associated topic, the callback function runs, and the published data appears in the
argument message_holder. (This message holder must be of a type compatible with the
message type published on the topic of interest, in this case std_msgs::Float64).


``
ros::Subscriber my_subscriber_objsect=n.subscribe("topic1",1,myCallback)
``
An object of type Subscriber is substantiated; There are three arguments used in instantiating the subscriber object. 
The first argument is the topic name; topic1 is chosen as the topic to which our minimal
publisher publishes. (For this example, we want our subscriber node to listen to the output
of our example publisher node.)
The second argument is the queue size. If the callback function has trouble keeping up
with published data, the data may be queued. In the present case, the queue size is set to
one. If the callback function cannot keep up with publications, messages will be lost by being
overwritten by newer messages. (Recall that in the first example, rostopic echo topic1
could not keep up with the 30 kHz rate of the original minimal publisher. Values displayed
skipped many intermediate messages.) For control purposes, typically only the most recent
sensor value published is of interest. If a sensor publishes faster than the callback function
can respond, there is no harm done in dropping messages, as only the most recent message
would be needed. In this (and many cases) a queue size of one message is all that is needed.
The third argument for instantiating the Subscriber object is the name of the callback
function that is to receive data from topic1. This argument has been set to myCallback,
which is the name of our callback function, described earlier. Through this line of code, we
associate our callback function with messages published on topic1.

After adding executable and target link libraries in to CMakeLists do the catkin_make in ros_ws folder. Then rosrun publisher and subscriber.

