load('Awesomedata.mat');
t = dataset(:,1); % time steps (s)
posL = dataset(:,2); % position of left wheel (m)
posR = dataset(:,3); % position of right wheel (m)
Gx = dataset(:,4); 
Gy = dataset(:,5);
Gz = dataset(:,6);
d = .25; % robot's wheel base (meters)

vel = 0.1; % forward velocity (m/s)
pos = [0 0 0]; % starting position 
Pos = []; % where the positions will be saved
psi = pi; % initializing yaw angle

for i = 1:size(t, 1)-1
    deltaL = posL(i+1)-posL(i); % change in position of left wheel (m)
    deltaR = posR(i+1)-posR(i); % change in position of right wheel (m)
    deltat = t(i+1)-t(i); % change in time (s)
    vL = deltaL/deltat; % velocity of left wheel (m/s)
    vR = deltaR/deltat; % velocity of right wheel (m/s)
    w = (vR-vL)/d; % angular velocity (radians/s)
    theta = atan2(-Gx(i),sqrt((Gy(i)^2)+(Gz(i)^2))); % pitch
    phi = atan2(Gy(i),Gz(i)); % roll
    deltapsi = w*deltat; 
    psi = psi+deltapsi; % yaw
    % rotation matrix
    Rxyz = [cos(theta)*cos(psi) cos(theta)*sin(psi) -sin(theta); ...
        cos(psi)*sin(theta)*sin(phi)-cos(phi)*sin(psi) cos(phi)*cos(psi)+sin(theta)*sin(phi)*sin(psi) cos(theta)*sin(phi);...
        cos(phi)*cos(psi)*sin(theta)+sin(phi)*sin(psi) cos(phi)*sin(theta)*sin(psi)-cos(psi)*sin(phi) cos(theta)*cos(phi)];
    roomVect = [1 0 0]*Rxyz; % vector in room's coordinate system
    avgDist = (deltaL + deltaR)/2; % average distance travelled
    pos = pos + avgDist.*roomVect; % new position
    Pos = [Pos pos']; % collection of new position points
end
x = Pos(1,:) % x positions
y = Pos(2,:) % y positions
z = Pos(3,:) % z positions
plot3(x,y,z,'bo')

