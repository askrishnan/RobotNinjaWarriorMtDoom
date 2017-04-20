%Making our ros objects 
pub = rospublisher('/raw_vel');
msg = rosmessage(pub);
%time object
time = tic;
run = 1;
d = .25; % meters
r = d/2
vel = .1
w = vel/r
theta = pi

sub_bump = rossubscriber('/bump');
while 1
    bumpMessage = receive(sub_bump);
    if any(bumpMessage.Data)
        msg.Data=[0,0];
        send(pub, msg);
        break
    end
end

while run == 1
%define t as our current time
t = toc(time);
    if t < theta/w
        vl = vel % left wheel speed (meters/second)
        vr = -1*vel % right wheel speed (meters/second)
        msg.Data = [vl, vr];
        send(pub, msg);
    else
        break
    end

end

msg.Data = [0, 0];
send(pub, msg);