
TopicList=ros2("topic","list");
NodeList=ros2("node","list");

ScanNode = ros2node("/scan");
OdomNode = ros2node("/turtlebot3_joint_state");
ControllerNode = ros2node("/turtlebot3_diff_drive");

LaserSub = ros2subscriber(ScanNode,"/scan");
pause(1);
scanData = receive(LaserSub,10);
Points360 = LaserSub.LatestMessage.ranges;

odomSub = ros2subscriber(OdomNode, "/odom");

[velPub, velMsg] = ros2publisher(ControllerNode, "/cmd_vel", "geometry_msgs/Twist");

% Publish velocity commands
velMsg.linear.x = 0;
velMsg.angular.z = 0;
send(velPub,velMsg);
x_0 = odomSub.LatestMessage.pose.pose.position.x;
y_0 = odomSub.LatestMessage.pose.pose.position.y;

