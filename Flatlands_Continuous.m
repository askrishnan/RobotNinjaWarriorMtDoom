syms x(t) y(t)
a = .01; % m/s
odex = diff(x)==a*(y-2*x-2);
odey = diff(y)==a*(x-2*y-2);

odes = [odex; odey];

cond1 = x(0)==4;
cond2 = y(0)==1;
conds = [cond1; cond2];
[xSol(t), ySol(t)] = dsolve(odes, conds)
fplot(xSol(t),ySol(t), [0 500])
axis([-4 4 -4 4])
xlabel('X axis')
ylabel('Y axis')
hold on

% Parameters
d = .25*3.28084; % wheel base (meters)
syms t
dilation = .5;
shape = sym([xSol(t); ySol(t); 0]);
T = diff(shape);
MagT = norm(T);
That = T./MagT;
N = diff(That);
w = cross(That, N);
w = w(3);
% Send wheel speeds via ROS
vel = norm(diff(shape));
o = matlabFunction(w);
velocity = matlabFunction(vel);
pub = rospublisher('/raw_vel');
sub  = rossubscriber('/encoders');
msg = rosmessage(pub);
time = tic;
acc = 1;
threshold = 0.1; 
while acc == 1
t = toc(time);
if t < 32
omega = o(t)% angular velocity (radians/second)
% velocity (meters/second)
test = velocity(t);

R = test/omega;
vl = test - omega*d/2 % left wheel speed (meters/second)
vr = test + omega*d/2% right wheel speed (meters/second)

msg.Data = [vl, vr];
send(pub, msg);
break;
end

end
msg.Data = [0, 0];
send(pub, msg);

