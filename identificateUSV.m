SampleTime = mean(diff(USV.time));

%% Longitudinal Feed Forward Table
speedCommandDelay = 10;                         % SET ME!!!
speedCommandLevelSteps = [20 40 60 80];         % SET ME!!!
speedStep = 0.05;                               % SET ME!!!
% maxSpeed = 3.1;                                 % Optional

if not(exist('maxSpeed', 'var'))
    maxSpeed = max(USV.longSpeed.Data);
end

commandTol = 0.5;
deltaTimeTol = SampleTime*1.05;

speedLevelSteps = cell(size(speedCommandLevelSteps));
speedLevelStepsWeight = cell(size(speedCommandLevelSteps));

fig1 = figure('Name', 'FeedForward Speed Gains Identification');
fig1.Position = [100 100 960 540];
plot(USV.speedCmd/20);
hold on
plot(USV.longSpeed);
title('Vehicle Longitudinal Dynamic Response');
xlabel('Time [s]');
ylabel('Level [1-5] - Speed [m/s]');
legendFields = {'Command','Speed'};

xlimLowLong = inf;
xlimHighLong = -inf;
colorMap = parula(length(speedCommandLevelSteps));
for commandLevel = speedCommandLevelSteps
    usefulRange = abs(squeeze(USV.speedCmd.Data)-commandLevel) < commandTol & ...
                  abs(squeeze(USV.yawRateCmd.Data)) < commandTol;

    startUsefulPeriod = [diff(USV.time(usefulRange)).'; inf] < deltaTimeTol;
    startUsefulPeriod = startUsefulPeriod & [true; not(startUsefulPeriod(1:end-1))];
    startUsefulPeriodIndex = find(startUsefulPeriod);
    stopUsefulPeriodIndex = [startUsefulPeriodIndex(2:end)-1; length(startUsefulPeriod)];

    usefulSpeedRange = squeeze(USV.longSpeed.Data(usefulRange));
    usefulTimeRange = USV.time(usefulRange);
    for i = 1:length(startUsefulPeriodIndex)
        usefulSpeedPeriod = usefulSpeedRange(startUsefulPeriodIndex(i):stopUsefulPeriodIndex(i));
        usefulTimePeriod = usefulTimeRange(startUsefulPeriodIndex(i):stopUsefulPeriodIndex(i));
        usefulSpeedPeriod = usefulSpeedPeriod(ceil(speedCommandDelay/SampleTime):end);
        usefulTimePeriod = usefulTimePeriod(ceil(speedCommandDelay/SampleTime):end);
        
        if not(isempty(usefulTimePeriod))       
            plot(usefulTimePeriod, usefulSpeedPeriod,'LineWidth', 2, 'color', colorMap(speedCommandLevelSteps == commandLevel,:));
            speedLevelSteps{speedCommandLevelSteps == commandLevel} = [speedLevelSteps{speedCommandLevelSteps == commandLevel} ...
                                                                       mean(usefulSpeedPeriod)];
            speedLevelStepsWeight{speedCommandLevelSteps == commandLevel} = [speedLevelStepsWeight{speedCommandLevelSteps == commandLevel} ...
                                                                             length(usefulTimePeriod)];
            legendFields{1+length(legendFields)} = [ 'commandLevel ' num2str(commandLevel)];
            xlimLowLong = min(xlimLowLong, usefulTimePeriod(1));
            xlimHighLong = max(xlimHighLong, usefulTimePeriod(end));
        end
    end
    
    
end
clear commandLevel colorMap usefulRange startUsefulPeriod startUsefulPeriodIndex stopUsefulPeriodIndex
clear usefulSpeedRange usefulTimeRange i usefulSpeedPeriod usefulTimePeriod

legend(legendFields, 'Location', 'northwest');
xlim([xlimLowLong xlimHighLong]);
clear legendFields
orient(fig1,'landscape')
print(fig1,'FeedForwardSpeedGainsIdentification.pdf','-dpdf')

% Longitudinal FF Table: Result
speedMeanLevelSteps = zeros(size(speedCommandLevelSteps));
for commandLevel = speedCommandLevelSteps
    for i = 1:length(speedLevelSteps{commandLevel == speedCommandLevelSteps})
        speedMeanLevelSteps(commandLevel == speedCommandLevelSteps) = ...
            sum(speedLevelSteps{commandLevel == speedCommandLevelSteps}.*...
                speedLevelStepsWeight{commandLevel == speedCommandLevelSteps}./...
                sum(speedLevelStepsWeight{commandLevel == speedCommandLevelSteps}));
    end
end

fig2 = figure('Name', 'FeedForward Speed Gain Table');
fig2.Position = [100 100 960 540];
hold on
colorMap = parula(length(speedCommandLevelSteps));
for commandLevel = speedCommandLevelSteps
    for i = 1:length(speedLevelSteps{commandLevel == speedCommandLevelSteps})
        plot(commandLevel, speedLevelSteps{commandLevel == speedCommandLevelSteps}(i), ...
             '*', 'LineWidth', 1.5, 'color', colorMap(speedCommandLevelSteps == commandLevel,:));
    end
end
plot(0, 0, ...
     'k*', 'LineWidth', 1.5);
plot(100, maxSpeed, ...
     'k*', 'LineWidth', 1.5);
% FFspeedCommands = 0:100;
% desideredSpeeds = interp1([0 speedCommandLevelSteps 100], ...
%                           [0 speedMeanLevelSteps maxSpeed], ...
%                           FFspeedCommands, 'pchip');
desideredSpeeds = 0:speedStep:maxSpeed;
FFspeedCommands = interp1([0 speedMeanLevelSteps maxSpeed],...
                          [0 speedCommandLevelSteps 100], ...
                          desideredSpeeds, 'pchip');
plot(FFspeedCommands, desideredSpeeds, 'ok');
clear commandLevel colorMap i
title('FeedForward Speed Table');
xlabel('u_+ | Speed Command Level [%]');
ylabel('Speed [m/s]');
orient(fig2,'landscape')
print(fig2,'FeedForwardSpeedTable.pdf','-dpdf')

clear speedLevelSteps speedLevelStepsWeight
clear commandTol deltaTimeTol

clear speedMeanLevelSteps

%% Lateral Feed Forward Table
yawRateCommandDelay = 5;                                % SET ME!!!
yawRateCommandLevelSteps = [40 60 -20 -40 -60];         % SET ME!!!
yawRateStep = 0.02;                                     % SET ME!!!
% maxYawRate = 0.50;                                      % Optional
% minYawRate = -0.52;                                     % Optional

if not(exist('maxYawRate', 'var'))
    maxYawRate = max(USV.yawRate.Data);
end
if not(exist('minYawRate', 'var'))
    minYawRate = min(USV.yawRate.Data);
end

commandTol = 0.5;
deltaTimeTol = SampleTime*1.05;

yawRateLevelSteps = cell(size(yawRateCommandLevelSteps));
yawRateLevelStepsWeight = cell(size(yawRateCommandLevelSteps));

fig3 = figure('Name', 'FeedForward YawRate Gains Identification');
fig3.Position = [100 100 960 540];
plot(USV.yawRateCmd/20);
hold on
plot(USV.yawRate*18/pi);
title('Vehicle Lateral Dynamic Response');
xlabel('Time [s]');
ylabel('Level [1-5] - Yaw Rate [10°/sec]');
legendFields = {'Command', 'Yaw Rate'};

xlimLowLat = inf;
xlimHighLat = -inf;
colorMap = parula(length(yawRateCommandLevelSteps));
for commandLevel = yawRateCommandLevelSteps
    usefulRange = abs(squeeze(USV.yawRateCmd.Data)-commandLevel) < commandTol & ...
                  abs(squeeze(USV.speedCmd.Data)) < commandTol;

    startUsefulPeriod = [diff(USV.time(usefulRange)).'; inf] < deltaTimeTol;
    startUsefulPeriod = startUsefulPeriod & [true; not(startUsefulPeriod(1:end-1))];
    startUsefulPeriodIndex = find(startUsefulPeriod);
    stopUsefulPeriodIndex = [startUsefulPeriodIndex(2:end)-1; length(startUsefulPeriod)];

    usefulYawRateRange = squeeze(USV.yawRate.Data(usefulRange));
    usefulTimeRange = USV.time(usefulRange);
    for i = 1:length(startUsefulPeriodIndex)
        usefulYawRatePeriod = usefulYawRateRange(startUsefulPeriodIndex(i):stopUsefulPeriodIndex(i));
        usefulTimePeriod = usefulTimeRange(startUsefulPeriodIndex(i):stopUsefulPeriodIndex(i));
        usefulYawRatePeriod = usefulYawRatePeriod(ceil(yawRateCommandDelay/SampleTime):end);
        usefulTimePeriod = usefulTimePeriod(ceil(yawRateCommandDelay/SampleTime):end);
        
        if not(isempty(usefulTimePeriod))       
            plot(usefulTimePeriod, usefulYawRatePeriod*18/pi,'LineWidth', 2, 'color', colorMap(yawRateCommandLevelSteps == commandLevel,:));

            yawRateLevelSteps{yawRateCommandLevelSteps == commandLevel} = [yawRateLevelSteps{yawRateCommandLevelSteps == commandLevel} ...
                                                                           mean(usefulYawRatePeriod)];
            yawRateLevelStepsWeight{yawRateCommandLevelSteps == commandLevel} = [yawRateLevelStepsWeight{yawRateCommandLevelSteps == commandLevel} ...
                                                                                 length(usefulTimePeriod)];
            legendFields{1+length(legendFields)} = [ 'commandLevel ' num2str(commandLevel)];
            xlimLowLat = min(xlimLowLat, usefulTimePeriod(1));
            xlimHighLat = max(xlimHighLat, usefulTimePeriod(end));
        end
        
    end
    
end
clear commandLevel colorMap usefulRange startUsefulPeriod startUsefulPeriodIndex stopUsefulPeriodIndex
clear usefulYawRateRange usefulTimeRange i usefulYawRatePeriod usefulTimePeriod

legend(legendFields);
xlim([xlimLowLat xlimHighLat]);
clear legendFields
orient(fig3,'landscape')
print(fig3,'FeedForwardYawRateGainsIdentification.pdf','-dpdf')

% Lateral FF Table: Result
yawRateMeanLevelSteps = zeros(size(yawRateCommandLevelSteps));
for commandLevel = yawRateCommandLevelSteps
    for i = 1:length(yawRateLevelSteps{commandLevel == yawRateCommandLevelSteps})
        yawRateMeanLevelSteps(commandLevel == yawRateCommandLevelSteps) = ...
            sum(yawRateLevelSteps{commandLevel == yawRateCommandLevelSteps}.*...
                yawRateLevelStepsWeight{commandLevel == yawRateCommandLevelSteps}./...
                sum(yawRateLevelStepsWeight{commandLevel == yawRateCommandLevelSteps}));
    end
end
clear commadLevel i

fig4 = figure('Name', 'FeedForward Speed Gain Table');
fig4.Position = [100 100 960 540];
hold on
colorMap = parula(length(yawRateCommandLevelSteps));
for commandLevel = yawRateCommandLevelSteps
    for i = 1:length(yawRateLevelSteps{commandLevel == yawRateCommandLevelSteps})
        plot(commandLevel, yawRateLevelSteps{commandLevel == yawRateCommandLevelSteps}(i)*18/pi, ...
             '*', 'LineWidth', 1.5, 'color', colorMap(yawRateCommandLevelSteps == commandLevel,:));
    end
end
clear commandLevel i colorMap
plot(0, 0, ...
     'k*', 'LineWidth', 1.5);
plot(-100, minYawRate*18/pi, ...
     'k*', 'LineWidth', 1.5);
plot(100, maxYawRate*18/pi, ...
     'k*', 'LineWidth', 1.5);
% FFyawRateCommands = -100:100;
% desideredYawRates = interp1([0 yawRateCommandLevelSteps -100 100], ...
%                             [0 yawRateMeanLevelSteps minYawRate maxYawRate], ...
%                             FFyawRateCommands, 'pchip');
desideredYawRates = minYawRate:yawRateStep:maxYawRate;
FFyawRateCommands = interp1([0 yawRateMeanLevelSteps minYawRate maxYawRate], ...
                          [0 yawRateCommandLevelSteps -100 100], ...
                          desideredYawRates, 'pchip');
plot(FFyawRateCommands, desideredYawRates*18/pi, 'ok');
title('FeedForward YawRate Table');
xlabel('u_- | Steer Command Level [%]');
ylabel('Yaw Rate [10°/sec]');
orient(fig4,'landscape')
print(fig4,'FeedForwardSpeedGainTable.pdf','-dpdf')

clear yawRateLevelSteps yawRateLevelStepsWeight
clear commandTol deltaTimeTol

% Lateral FF Table: Fixed Result (fill symmetrical missing point)
yawRateCommandLevelStepsFixed = yawRateCommandLevelSteps;
yawRateMeanLevelStepsFixed = yawRateMeanLevelSteps;
fig5 = figure('Name', 'FeedForward Speed Gain Table Fixed');
fig5.Position = [100 100 960 540];
hold on
colorMap = parula(length(yawRateCommandLevelSteps));
for commandLevel = yawRateCommandLevelSteps
    plot(commandLevel, yawRateMeanLevelSteps(commandLevel == yawRateCommandLevelSteps)*18/pi, ...
         '*', 'LineWidth', 1.5, 'color', colorMap(yawRateCommandLevelSteps == commandLevel,:));
    if not(any(-commandLevel == yawRateCommandLevelSteps))
        yawRateCommandLevelStepsFixed = [yawRateCommandLevelStepsFixed ...
                                         -commandLevel];
        yawRateMeanLevelStepsFixed = [yawRateMeanLevelStepsFixed ...
                                      -yawRateMeanLevelSteps(commandLevel == yawRateCommandLevelSteps)];
        plot(-commandLevel, -yawRateMeanLevelSteps(commandLevel == yawRateCommandLevelSteps)*18/pi, ...
             'r*', 'LineWidth', 1.5);
    end
end
clear commandLevel
plot(0, 0, ...
     'k*', 'LineWidth', 1.5);
plot(-100, minYawRate*18/pi, ...
     'k*', 'LineWidth', 1.5);
plot(100, maxYawRate*18/pi, ...
     'k*', 'LineWidth', 1.5);
% FFyawRateCommands = -100:100;
% desideredYawRates = interp1([0 yawRateCommandLevelStepsFixed -100 100], ...
%                             [0 yawRateMeanLevelStepsFixed minYawRate maxYawRate], ...
%                             FFyawRateCommands, 'pchip');
desideredYawRates = minYawRate:yawRateStep:maxYawRate;
FFyawRateCommands = interp1([0 yawRateMeanLevelStepsFixed minYawRate maxYawRate], ...
                          [0 yawRateCommandLevelStepsFixed -100 100], ...
                          desideredYawRates, 'pchip'); 
plot(FFyawRateCommands, desideredYawRates*18/pi, 'ok');
clear commandLevel colorMap
title('FeedForward YawRate Table Fixed');
xlabel('u_- | Steer Command Level [%]');
ylabel('Yaw Rate [10°/sec]');
orient(fig5,'landscape')
print(fig5,'FeedForwardSpeedGainTableFixed.pdf','-dpdf')

clear yawRateCommandLevelStepsFixed yawRateMeanLevelSteps yawRateMeanLevelStepsFixed

%% Save FF Tables Results
writematrix([desideredSpeeds.' FFspeedCommands.'], 'FFspeed_USV.csv');
writematrix([desideredYawRates.' FFyawRateCommands.'], 'FFyawRate_USV.csv');

%% Longitudinal Dynamic
rangeIndex = USV.time>xlimLowLong & USV.time<xlimHighLong & ...
             squeeze(USV.speedCmd.Data).' > 30 & ...
             squeeze(USV.speedCmd.Data).' < 70;
idU = interp1(FFspeedCommands, desideredSpeeds, squeeze(USV.speedCmd.Data(rangeIndex)));
idY = squeeze(USV.longSpeed.Data(rangeIndex));
idData = iddata(idY, idU, SampleTime);
longitudinalP1D  = procest(idData,'P1D');
fig6 = figure('Name', 'Longitudinal Dynamic Identification');
fig6.Position = [100 100 960 540];
compare(idData,longitudinalP1D);
hold on
plot((1:length(idU)).*SampleTime, idU, 'DisplayName','Speed Input');
hold off
ylabel('Speed [m/s]');
clear rangeIndex idU idY idData
legend('Location','northwest');
orient(fig6,'landscape')
print(fig6,'LongitudinalDynamicIdentification.pdf','-dpdf')

rangeIndex = USV.time>xlimLowLong & USV.time<xlimHighLong;
idU = interp1(FFspeedCommands, desideredSpeeds, squeeze(USV.speedCmd.Data(rangeIndex)), 'pchip');
idY = squeeze(USV.longSpeed.Data(rangeIndex));
idData = iddata(idY, idU, SampleTime);
fig7 = figure('Name', 'Longitudinal Dynamic Validation');
fig7.Position = [100 100 960 540];
compare(idData,longitudinalP1D);
hold on
plot((1:length(idU)).*SampleTime, idU, 'DisplayName','Speed Input');
hold off
ylabel('Speed [m/s]');
clear rangeIndex idU idY idData
legend('Location','northwest');
orient(fig7,'landscape')
print(fig7,'LongitudinalDynamicValidation.pdf','-dpdf')

clear xlimLowLong xlimHighLong

%% Lateral Dynamic
rangeIndexA = USV.time>xlimLowLat & USV.time<xlimHighLat & ...
              squeeze(USV.yawRateCmd.Data).' > 30 & squeeze(USV.yawRateCmd.Data).' < 70;
rangeIndexB = USV.time>xlimLowLat & USV.time<xlimHighLat & ...
              squeeze(USV.yawRateCmd.Data).' > -70 & squeeze(USV.yawRateCmd.Data).' < -30;
if sum(double(rangeIndexA))>sum(double(rangeIndexB))
    rangeIndex = rangeIndexA;
else
    rangeIndex = rangeIndexB;
end
clear rangeIndexA rangeIndexB
idU = interp1(FFyawRateCommands, desideredYawRates, squeeze(USV.yawRateCmd.Data(rangeIndex)), 'pchip');
idY = squeeze(USV.yawRate.Data(rangeIndex));
idData = iddata(idY, idU, SampleTime);
lateralP1D  = procest(idData,'P1D');
fig8 = figure('Name', 'Lateral Dynamic Identification');
fig8.Position = [100 100 960 540];
compare(idData,lateralP1D);
hold on
plot((1:length(idU)).*SampleTime, idU, 'DisplayName','Yaw Rate Input');
hold off
ylabel('Yaw Rate [rad/sec]');
clear rangeIndex idU idY idData
orient(fig8,'landscape')
print(fig8,'LateralDynamicIdentification.pdf','-dpdf')

rangeIndex = USV.time>xlimLowLat & USV.time<xlimHighLat;
idU = interp1(FFyawRateCommands, desideredYawRates, squeeze(USV.yawRateCmd.Data(rangeIndex)), 'pchip');
idY = squeeze(USV.yawRate.Data(rangeIndex));
idData = iddata(idY, idU, SampleTime);
fig9 = figure('Name', 'Lateral Dynamic Validation');
fig9.Position = [100 100 960 540];
compare(idData,lateralP1D);
hold on
plot((1:length(idU)).*SampleTime, idU, 'DisplayName','Yaw Rate Input');
hold off
ylabel('Yaw Rate [rad/sec]');
clear rangeIndex idU idY idData
orient(fig9,'landscape')
print(fig9,'LateralDynamicValidation.pdf','-dpdf')

clear xlimLowLat xlimHighLat

%% Save Dynamic Parameters
writematrix(longitudinalP1D.Tp1, 'TAUspeed_USV.txt');
writematrix(lateralP1D.Tp1, 'TAUyawRate_USV.txt');

%% Clear
clear speedCommandDelay speedCommandLevelSteps speedStep maxSpeed
clear yawRateCommandDelay yawRateCommandLevelSteps yawRateStep maxYawRate minYawRate