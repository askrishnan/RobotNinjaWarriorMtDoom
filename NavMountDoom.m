% Initialize publishers and subscribers
pub = rospublisher('/raw_vel');
msg = rosmessage(pub);
encs = receive(sub_encoders);
sub_accel = rossubscriber('/accel');


% For turning the robot
d = .25; % robot's wheel base (meters)
w = .3; % angular velocity (radians/second)
vlr = -(w*d)/2; % left wheel speed (meters/second)
vrr = (w*d)/2; % right wheel speed (meters/second)
threshold = .05; % threshold for y because it won't end up equalling exactly zero 

for i = 1:40
    i;
    while 1
        % To turn the robot in the right direction
        % Save accelerometer data
        accelMsg = receive(sub_accel);
        accel = accelMsg.Data;
        x = accel(1); 
        y = accel(2); 
        z = accel(3); 

        % Use the y to figure out the path of steepest ascent
        % Check y every time the Neato stops

        % If y is within the threshold, stop the robot (it's at the top!)
        if y < threshold && y > -threshold
            msg.Data = [0,0];
            send(pub, msg);
            break
        end
        % If y is greater than 0, turn left
        if y > 0
            vl = vlr;
            vr = vrr;
        % If y is less than 0, turn right
        else
            vl = -vlr;
            vr = -vrr;    
        end

        % Send data to wheels so the Neato rotates
        msg.Data = [vl,vr];
        send(pub, msg);
    end
    
        slope = sqrt((1 - z^2)/(z^2)); % slope @ steepest ascent
        pitch = atan2(-x, sqrt(y^2+z^2)); % angle the robot is jacked at
        
        % Move forward
        move = slope-pitch; % distance to travel 
        vel = 0.1; % forward velocity (m/s)
        lambda = .1; % step size
        move_time = move/vel*lambda+.15;
        msg.Data = [vel,vel]; % Neato moves forward straight
        send(pub, msg);
        pause(move_time); % moves robot until we want it to stop

        msg.Data = [0,0]; % stops Neato
        send(pub, msg);

end

