clear
pub = rospublisher('/raw_vel');
sub = rossubscriber('/encoders');
msg = rosmessage(pub);

sub_bump = rossubscriber('/bump');

% Define the function
func = @(x) x(1)*x(2) - x(1)^2 - x(2)^2 - 2*x(1)- 2*x(2) + 4;
% Define the gradient vector
Gradient = @(x) [x(2)-2*x(1)-2; x(1)-2*x(2)-2];
% Test at the initial point
d = .25; % meters
x = [4; 1]; % beginning point
phi = pi/2; % radians
lambda = 1/10; % step size
delta = 1.5; % change in step size
vel = 0.1; 
w = vel/(d/2); % angular velocity
vlr = -(w*d)/2; % left wheel speed (meters/second)
vrr = (w*d)/2; % right wheel speed (meters/second)
y = 0; % for while loop
time = tic;

for y = 1:7
gradient = Gradient(x); % creates gradient from initial point
shift = lambda.*gradient; % multiplies step size by gradient
length = norm(shift); % gives length of resulting vector
x = x + shift; % shifted position (where the robot is)
lambda = lambda*delta; % decreases step size a little
theta = atan2(gradient(2),gradient(1)); % angle robot is currently at
delta_phi = theta-phi; % pre-movement angle
if delta_phi < -pi
    delta_phi = delta_phi + 2*pi; % makes sure the robot doesn't make a full rotation and instead makes a small rotation
end


movetime = length/(3.28084*vel); %m/m*s* (m/s *f/m== f/s /f)

t = toc(time);
bumpMessage = receive(sub_bump);
if any(bumpMessage.Data)
    msg.Data=[0,0];
    send(pub, msg);
end

turn_time = delta_phi/w; % how long it should take for the robot to turn
if turn_time < 0 
    turn_time = -turn_time; % makes turn time positive for rotation
    Vrr = -vrr; % switches wheel velocities
    Vlr = - Vrr; % switches wheel velocities
else 
    Vlr = vlr;
    Vrr = vrr;
end

msg.Data = [Vlr, Vrr];
send(pub, msg);

pause(turn_time)
    
msg.Data=[0,0];
send(pub, msg);

vl = vel;
vr = vel;
msg.Data = [vl, vr];

send(pub, msg);
pause(movetime)
msg.Data=[0,0];
send(pub, msg);

phi = theta; % updates current angle
end    

msg.Data = [0, 0];
send(pub, msg);
