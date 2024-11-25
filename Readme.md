https://it.mathworks.com/help/ros/ug/sign-following-robot-using-ros2-matlab.html
https://it.mathworks.com/help/ros/ug/control-nvidia-carter-robot-in-isaac-sim-using-ros2.html
https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-Spawn-and-delete

 https://it.mathworks.com/help/ros/ug/sign-following-robot-using-ros2-matlab.html

 tfSub = ros2subscriber(node, '/tf', 'geometry_msgs/TransformStamped', @tfCallback);

gazeboVMDevice = ros2device('192.168.192.129','user','password');
gazeboVMDevice.AvailableNodes


 imgSub = ros2subscriber(n, "/camera/image_raw","sensor_msgs/Image","Reliability","besteffort","Durability","volatile","Depth",5);

odomSub = ros2subscriber(n, "/odom","nav_msgs/Odometry","Reliability","besteffort","Durability","volatile","Depth",5);

[velPub, velMsg] = ros2publisher(n, "/cmd_vel", "geometry_msgs/Twist","Reliability","besteffort","Durab

gazebo = ExampleHelperGazeboCommunicator; % create communication object

 cil = ExampleHelperGazeboModel("Cil")
cilLink = addLink(cil,"cylinder",[1 0.2],"color",[1 0 0 1]) %type, rad, color
spawnModel(gazebo,cil,[6 2 1])


ball1 = ExampleHelperGazeboModel("Ball")
sphereLink = addLink(ball1,"sphere",0.1,"color",[0 0 1 1]) %type, rad, color
spawnModel(gazebo,ball1,[5 0 0.1])

ball2 = ExampleHelperGazeboModel("Ball2")
sphereLink = addLink(ball2,"sphere",0.1,"color",[0 1 1 1]) %type, rad, color
spawnModel(gazebo,ball2,[6 -1 0.1])




detectNode = ros2node("/scan");
laserSub = ros2subscriber(detectNode,"/scan");
laserSub.LatestMessage.ranges
