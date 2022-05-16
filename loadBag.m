bagFileName = '../20220414_NEMI_pm_openloop_thruster_test.bag';

%% Load Data
bag = rosbag(bagFileName);

% select topics
bSelGPS = select(bag,'Topic','/xsens_id0_gps');
bSelIMU = select(bag,'Topic','/xsens_id0_imu');
bSelOdom = select(bag,'Topic','/xsens_id0_odom');
bSelThruster = select(bag,'Topic','/ros2can/cmd_thruster');

% gps data manipulation
mSelGPS = readMessages(bSelGPS,'DataFormat','struct');
mSelGPS = [mSelGPS{:}];
hSelGPS = [mSelGPS.Header];
stampSelGPS = [hSelGPS.Stamp];
GPS.time = double([stampSelGPS.Sec]) + double([stampSelGPS.Nsec])./1e9;
lla0 = [41.71466 12.70488, 372]; % center of nemi lake
lla = lla2enu([[mSelGPS.Latitude]' [mSelGPS.Longitude]' [mSelGPS.Altitude]'], lla0, 'flat');
GPS.X = timeseries(lla(:,1).', GPS.time, 'Name', 'GPS X');
GPS.X = setinterpmethod(GPS.X, 'zoh');
GPS.Y = timeseries(lla(:,2).', GPS.time, 'Name', 'GPS Y');
GPS.Y = setinterpmethod(GPS.Y, 'zoh');
GPS.Z = timeseries(lla(:,3).', GPS.time, 'Name', 'GPS Z');
GPS.Z = setinterpmethod(GPS.Z, 'zoh');
clear mSelGPS hSelGPS stampSelGPS lla0 lla

% imu data manipulation
mSelIMU = readMessages(bSelIMU,'DataFormat','struct');
mSelIMU = [mSelIMU{:}];
hSelIMU = [mSelIMU.Header];
stampSelIMU = [hSelIMU.Stamp];
IMU.time = double([stampSelIMU.Sec]) + double([stampSelIMU.Nsec])./1e9;
orientationIMU = [mSelIMU.Orientation];
eulerIMU = quat2eul([[orientationIMU.W]' [orientationIMU.X]' [orientationIMU.Y]' [orientationIMU.Z]']);
IMU.yaw = timeseries(eulerIMU(:,1).', IMU.time, 'Name', 'IMU Yaw');
IMU.yaw = setinterpmethod(IMU.yaw, 'zoh');
IMU.pitch = timeseries(eulerIMU(:,2).', IMU.time, 'Name', 'IMU Pitch');
IMU.pitch = setinterpmethod(IMU.pitch, 'zoh');
IMU.roll = timeseries(eulerIMU(:,3).', IMU.time, 'Name', 'IMU Roll');
IMU.roll = setinterpmethod(IMU.roll, 'zoh');
linearAccIMU = [mSelIMU.LinearAcceleration];
IMU.accX = timeseries([linearAccIMU.X], IMU.time, 'Name', 'IMU Acc X');
IMU.accX = setinterpmethod(IMU.accX, 'zoh');
IMU.accY = timeseries([linearAccIMU.Y], IMU.time, 'Name', 'IMU Acc Y');
IMU.accY = setinterpmethod(IMU.accY, 'zoh');
IMU.accZ = timeseries([linearAccIMU.Z], IMU.time, 'Name', 'IMU Acc Z');
IMU.accZ = setinterpmethod(IMU.accZ, 'zoh');
angularVelIMU = [mSelIMU.AngularVelocity];
IMU.rollRate = timeseries([angularVelIMU.X], IMU.time, 'Name', 'IMU Roll Rate');
IMU.rollRate = setinterpmethod(IMU.rollRate, 'zoh');
IMU.pitchRate = timeseries([angularVelIMU.Y], IMU.time, 'Name', 'IMU Pitch Rate');
IMU.pitchRate = setinterpmethod(IMU.pitchRate, 'zoh');
IMU.yawRate = timeseries([angularVelIMU.Z], IMU.time, 'Name', 'IMU Yaw Rate');
IMU.yawRate = setinterpmethod(IMU.yawRate, 'zoh');
clear mSelIMU hSelIMU stampSelIMU orientationIMU eulerIMU linearAccIMU angularVelIMU

% odom data manipulation
mSelOdom = readMessages(bSelOdom,'DataFormat','struct');
mSelOdom = [mSelOdom{:}];
hSelOdom = [mSelOdom.Header];
stampSelOdom = [hSelOdom.Stamp];
Odom.time = double([stampSelOdom.Sec]) + double([stampSelOdom.Nsec])./1e9;
% poseOdom = [mSelOdom.Pose];
% poseOdom = [poseOdom.Pose];
% orientationOdom = [poseOdom.Orientation];
% eulerOdom = quat2eul([[orientationOdom.W]' [orientationOdom.X]' [orientationOdom.Y]' [orientationOdom.Z]']);
% Odom.yaw = timeseries(eulerOdom(:,1).', Odom.time, 'Name', 'Odom Yaw');
% Odom.yaw = setinterpmethod(Odom.yaw, 'zoh');
% Odom.pitch = timeseries(eulerOdom(:,2).', Odom.time, 'Name', 'Odom Pitch');
% Odom.pitch = setinterpmethod(Odom.pitch, 'zoh');
% Odom.roll = timeseries(eulerOdom(:,3).', Odom.time, 'Name', 'Odom Roll');
% Odom.roll = setinterpmethod(Odom.roll, 'zoh');
% positionOdom = [poseOdom.Position];
% Odom.X = timeseries([positionOdom.X], Odom.time, 'Name', 'Odom X');
% Odom.X = setinterpmethod(Odom.X, 'zoh');
% Odom.Y = timeseries([positionOdom.Y], Odom.time, 'Name', 'Odom Y');
% Odom.Y = setinterpmethod(Odom.Y, 'zoh');
% Odom.Z = timeseries([positionOdom.Z], Odom.time, 'Name', 'Odom Z');
% Odom.Z = setinterpmethod(Odom.Z, 'zoh');
twistOdom = [mSelOdom.Twist];
twistOdom = [twistOdom.Twist];
linearVelOdom = [twistOdom.Linear];
Odom.velX = timeseries([linearVelOdom.X], Odom.time, 'Name', 'Odom Velocity X');
Odom.velX = setinterpmethod(Odom.velX, 'zoh');
Odom.velY = timeseries([linearVelOdom.Y], Odom.time, 'Name', 'Odom Velocity Y');
Odom.velY = setinterpmethod(Odom.velY, 'zoh');
Odom.velZ = timeseries([linearVelOdom.Z], Odom.time, 'Name', 'Odom Velocity Z');
Odom.velZ = setinterpmethod(Odom.velZ, 'zoh');
angularVelOdom = [twistOdom.Angular];
Odom.rollRate = timeseries([angularVelOdom.X], Odom.time, 'Name', 'Odom Roll Rate');
Odom.rollRate = setinterpmethod(Odom.rollRate, 'zoh');
Odom.pitchRate = timeseries([angularVelOdom.Y], Odom.time, 'Name', 'Odom Pitch Rate');
Odom.pitchRate = setinterpmethod(Odom.pitchRate, 'zoh');
Odom.yawRate = timeseries([angularVelOdom.Z], Odom.time, 'Name', 'Odom Yaw Rate');
Odom.yawRate = setinterpmethod(Odom.yawRate, 'zoh');
clear mSelOdom hSelOdom stampSelOdom poseOdom orientationOdom eulerOdom positionOdom twistOdom linearVelOdom angularVelOdom

% thruster data manipulation
mSelThruster = readMessages(bSelThruster,'DataFormat','struct');
mSelThruster = [mSelThruster{:}];
Thruster.time = double([bSelThruster.MessageList.Time].');
dataThruster = [mSelThruster.Data];
Thruster.left = timeseries(dataThruster(1,:), Thruster.time, 'Name', 'Left Motor');
Thruster.left = setinterpmethod(Thruster.left, 'zoh');
Thruster.right = timeseries(dataThruster(2,:), Thruster.time, 'Name', 'Right Motor');
Thruster.right = setinterpmethod(Thruster.right, 'zoh');
clear mSelThruster dataThruster

% clear bag data
clear bag bSelGPS bSelIMU bSelOdom bSelThruster

%% Clear

clear bagFileName