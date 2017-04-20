clear;
% Making our ROS objects 
pub = rospublisher('/raw_vel');
msg = rosmessage(pub);
sub_encoders = rossubscriber('/encoders');
encs = receive(sub_encoders);
data = encs.Data;

% Parameters
d = .25; % wheel base (meters)
syms t 
dilation = 5;
a = 0.4;
l = 0.4;
%Our cardioid equation
cardioid = sym([-2*a*(((l-cos(t/dilation))*cos(t/dilation) + (1-l))); 2*a*(((l-cos(t/dilation))*sin(t/dilation))); 0]);
%Finding the That vector
T = diff(cardioid);
MagT = norm(T);
That = T./MagT;
%Calculating the normal vector
N = diff(That);
%Calculating angular velocity
w = cross(That, N);
w = w(3);

%calculating total velocity
vel = norm(diff(cardioid));
o = matlabFunction(w);
velocity = matlabFunction(vel);

%time object
time = tic;
run = 1; 
x=o(1);
y=o(2);

oldleft = encs.Data(1);
oldright = encs.Data(2);
theta = 0;
position = [0; 0];
while 1
    %define t as our current time
    t = toc(time);
    
    if t < 19
        omega = o(t);% angular velocity (radians/second)
        test = velocity(t); % velocity (meters/second)
        vl = test - omega*d/2; % left wheel speed (meters/second)
        vr = test + omega*d/2; % right wheel speed (meters/second)
        msg.Data = [vl, vr];
        send(pub, msg);
        
        encs= receive(sub_encoders);
        newleft = encs.Data(1);
        newright = encs.Data(2);

        xt = position(1);
        yt = position(2);
        deltaL = newleft - oldleft;
        deltaR = newright - oldright;
        deltatheta = ((deltaR-deltaL)/d);
        R = (d/2)*((deltaL+deltaR)/(deltaR-deltaL));
        ICC = [xt-R*sin(theta); yt+R*cos(theta)];
        newpos = [cos(deltatheta) -sin(deltatheta); sin(deltatheta) cos(deltatheta)]*(position-ICC)+ICC;
        hold on
        plot(newpos(1),newpos(2),'bo');
        position = newpos;
        theta = theta + deltatheta;
        oldleft = newleft;
        oldright = newright;        
    else
        break;
    end
end
msg.Data = [0, 0];
send(pub, msg);