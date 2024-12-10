
TopicList=ros2("topic","list");
NodeList=ros2("node","list");

ScanNode = ros2node("/scan");
OdomNode = ros2node("/odom");
ControllerNode = ros2node("/odom");

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


MapNode = ros2node("/map");
MapSub = ros2subscriber(MapNode,"/map");
pause(0.5);
map_data = MapSub.LatestMessage.data;
map_matrix = reshape(map_data,[MapSub.LatestMessage.info.width,MapSub.LatestMessage.info.height]);
% subplot(2,2,[1,3])
% hold on
% imshow(map_matrix);

Map_metadata_Node = ros2node("/map_metadata");
Map_metadata_Sub = ros2subscriber(Map_metadata_Node,"/map_metadata");
% map_metadata_data = Map_metadata_Sub.LatestMessage.
% map_metadata_matrix = reshape(map_metadata_data,[Map_metadata_Sub.LatestMessage.width,Map_metadata_Sub.LatestMessage.height]);
% imshow(map_metadata_matrix)

