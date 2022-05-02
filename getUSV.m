SampleTime = 0.01;      % SET ME!!!

%% Pick useful Data for USV Identification
timeStart = max(Thruster.time(1) ,max(GPS.time(1), max(IMU.time(1), Odom.time(1))));
timeStop = min(Thruster.time(end) ,min(GPS.time(end), min(IMU.time(end), Odom.time(end))));

USV.time = timeStart:SampleTime:timeStop;
USV.X = resample(GPS.X, USV.time);
USV.Y = resample(GPS.Y, USV.time);
USV.yaw = resample(IMU.yaw, USV.time);
USV.velX = resample(Odom.velX, USV.time);
USV.velY = resample(Odom.velY, USV.time);
VGlobal = [squeeze(USV.velX.Data).'; squeeze(USV.velY.Data).'];
yaw = squeeze(USV.yaw.Data).';
VLocal = zeros(size(VGlobal));
for i = 1:size(VGlobal,2)
    Yawi = -yaw(1,i);
    Ri = [cos(Yawi) -sin(Yawi);...
          sin(Yawi)  cos(Yawi)];
    VLocal(:,i) = Ri*VGlobal(:,i);
end
USV.longSpeed = timeseries(VLocal(1,:), USV.time, 'Name', 'Longitudinal Speed');
USV.longSpeed = setinterpmethod(USV.longSpeed, 'zoh');
USV.latSpeed = timeseries(VLocal(2,:), USV.time, 'Name', 'Lateral Speed');
USV.latSpeed = setinterpmethod(USV.latSpeed, 'zoh');
USV.yawRate = resample(IMU.yawRate, USV.time);
USV.leftMotor = resample(Thruster.left, USV.time);
USV.rightMotor = resample(Thruster.right, USV.time);
USV.speedCmd = (USV.rightMotor + USV.leftMotor)./2;         % u+
USV.yawRateCmd = (USV.rightMotor - USV.leftMotor)./2;       % u-
clear timeStop VGlobal yaw VLocal i Yawi Ri

%% Reset USV Data time
USV.time = USV.time - timeStart;
USV.X.Time = USV.X.Time - timeStart;
USV.Y.Time = USV.Y.Time - timeStart;
USV.yaw.Time = USV.yaw.Time - timeStart;
USV.velX.Time = USV.velX.Time - timeStart;
USV.velY.Time = USV.velY.Time - timeStart;
USV.longSpeed.Time = USV.longSpeed.Time - timeStart;
USV.latSpeed.Time = USV.latSpeed.Time - timeStart;
USV.yawRate.Time = USV.yawRate.Time - timeStart;
USV.leftMotor.Time = USV.leftMotor.Time - timeStart;
USV.rightMotor.Time = USV.rightMotor.Time - timeStart;
USV.speedCmd.Time = USV.speedCmd.Time - timeStart;
USV.yawRateCmd.Time = USV.yawRateCmd.Time - timeStart;
clear timeStart

%% Clear

clear SampleTime