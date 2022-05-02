%% Plot USV Data
figure('Name', 'Position');
title('2-D Position')
hold on
plot(squeeze(USV.X.Data), squeeze(USV.Y.Data));
legend({'GPS'});
xlabel('X [m]');
ylabel('Y [m]');
axis equal

figure('Name', 'Velocity');
plot(USV.velX);
hold on
plot(USV.velY);
title('Global Velocity')
legend({'Odom VelX', 'Odom VelY'});
xlabel('Time [s]');
ylabel('Velocity [m/s]');

figure('Name', 'Speed');
plot(USV.longSpeed);
hold on
plot(USV.latSpeed);
title('Local Velocity')
legend({'Longitudinal Speed', 'Lateral Speed'});
xlabel('Time [s]');
ylabel('Velocity [m/s]');

figure('Name', 'Yaw Rate');
plot(USV.yawRate*180/pi)
title('Yaw Rate');
legend({'IMU Yaw Rate'});
xlabel('Time [s]');
ylabel('Angular Velocity [deg/sec]');

figure('Name', 'Motor');
plot(USV.leftMotor);
hold on
plot(USV.rightMotor);
title('Motors Thruster');
legend({'Left', 'Right'});
xlabel('Time [s]');
ylabel('Thruster [%]');

figure('Name', 'Command');
plot(USV.speedCmd);
hold on
plot(USV.yawRateCmd);
title('User Command');
legend({'Speed', 'YawRate'});
xlabel('Time [s]');
ylabel('Level [%]');

figure('Name', 'Longitudinal Dynamic');
plot(USV.speedCmd/20);
hold on
plot(USV.longSpeed);
title('Vehicle Longitudinal Dynamic Response');
legend({'Command', 'Speed'});
xlabel('Time [s]');
ylabel('Level [1-5] - Speed [m/s]');

figure('Name', 'Lateral Dynamic');
plot(USV.yawRateCmd/20);
hold on
plot(USV.yawRate*18/pi);
title('Vehicle Lateral Dynamic Response');
legend({'Command', 'Yaw Rate'});
xlabel('Time [s]');
ylabel('Level [1-5] - Yaw Rate [10°/sec]');