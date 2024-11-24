
Robot.Heading = Robot.Heading + Fuzzy_Local_Direction_ref;
Robot.Heading  = (Robot.Heading  + (360*(Robot.Heading <-180)) + (-360*(Robot.Heading >180)));

Xd(1,Step_Counter) = X(1,Step_Counter) + V * cosd(Robot.Heading);
X(1,Step_Counter+1) = Xd(1,Step_Counter);

Xd(2,Step_Counter) = X(3,Step_Counter) + V * sind(Robot.Heading);
X(3,Step_Counter+1) = Xd(2,Step_Counter);

X(5,Step_Counter+1) = Robot.Heading;

Dist2Goal(1, Step_Counter) = dist2goal([X(1,Step_Counter), X(3,Step_Counter)],X_g(:,Step_Counter));
