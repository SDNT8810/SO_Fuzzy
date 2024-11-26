
% Robot.Heading = Robot.Heading + Fuzzy_Local_Direction_ref;
% Robot.Heading  = (Robot.Heading  + (360*(Robot.Heading <-180)) + (-360*(Robot.Heading >180)));
% Xd(1,Step_Counter) = X(1,Step_Counter) + V * cosd(Robot.Heading);
% Xd(2,Step_Counter) = X(3,Step_Counter) + V * sind(Robot.Heading);

if (Gazebo_Sim == 1)
    velMsg.linear.x =  V;
    ntemp_w = (Fuzzy_Local_Direction_ref) * Omega / 180;
    % ntemp_w = sign(ntemp_w) * abs(Omega - abs(ntemp_w));
    if (isnan(ntemp_w))
        ntemp_w = 0;
    end
    ntemp_w = sign(ntemp_w) * min(abs(ntemp_w),Omega);
    % temp_w = (temp_w + ntemp_w) / 2;
    velMsg.angular.z = ntemp_w;
    Xd(5,Step_Counter+1) = Fuzzy_Local_Direction_ref;
    Xd(6,Step_Counter+1) = temp_w;
    send(velPub,velMsg);
    X(1,Step_Counter+1) = odomSub.LatestMessage.pose.pose.position.x;
    X(3,Step_Counter+1) = odomSub.LatestMessage.pose.pose.position.y;
    q1 = odomSub.LatestMessage.pose.pose.orientation.x;
    q2 = odomSub.LatestMessage.pose.pose.orientation.y;
    q3 = odomSub.LatestMessage.pose.pose.orientation.z;
    q4 = odomSub.LatestMessage.pose.pose.orientation.w;
    eulers = quat2eul([q1,q2,q3,q4]);
    Robot.Heading = (180/pi) * eulers(3);
    % pause(0.01)
else
    X(1,Step_Counter+1) = Xd(1,Step_Counter);
    X(3,Step_Counter+1) = Xd(2,Step_Counter);
end
    X(5,Step_Counter+1) = Robot.Heading;
    Dist2Goal(1, Step_Counter) = dist2goal([X(1,Step_Counter), X(3,Step_Counter)],X_g(:,Step_Counter));
