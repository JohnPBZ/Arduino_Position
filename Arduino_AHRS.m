% a = arduino('COM3', 'Uno', 'Libraries', 'I2C');
% fs = 50; % Sample Rate in Hz
% imu = mpu9250(a,'SampleRate',fs,'OutputFormat','matrix'); 
%===========================================================================

samplePeriod = 1/fs;
% GyroscopeNoise and AccelerometerNoise is determined from datasheet.
GyroscopeNoiseMPU9250 = 3.0462e-06; % GyroscopeNoise (variance) in units of rad/s
AccelerometerNoiseMPU9250 = 0.0061; % AccelerometerNoise (variance) in units of m/s^2
% viewer = HelperOrientationViewer('Title',{'IMU Filter'});
FUSE = imufilter('SampleRate',imu.SampleRate, 'GyroscopeNoise',GyroscopeNoiseMPU9250,'AccelerometerNoise', AccelerometerNoiseMPU9250);
stopTimer=100;
% Use imufilter to estimate orientation and update the viewer as the
% sensor moves for time specified by stopTimer
linVel=[0 0 0];
linAcc=[0 0 0];
linPos=[0 0 0];
imuGyro=[0 0 0];
imuAcc=[0 0 0];
i=2;

% order = 1;
% filtCutOff = 0.1;
% [b, a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high');
% freqz(b,a);

% fig = figure('NumberTitle', 'off', 'Name', '6DOF Animation');
% View = [30 20];
% Position = [9 39 1280 720];
% set(fig, 'Position', Position);
% set(gca, 'drawmode', 'fast');
% lighting phong;
% set(gcf, 'Renderer', 'zbuffer');
% hold on;
% axis equal;
% grid on;
% 
% orgHandle = plot3(0, 0, 0, 'k.');
% quivXhandle = quiver3(0, 0, 0, 1, 0, 0,  'r', 'ShowArrowHead', 'off', 'MaxHeadSize', 0.999999, 'AutoScale', 'off');
% quivYhandle = quiver3(0, 0, 0, 0, 1, 0,  'g', 'ShowArrowHead', 'off', 'MaxHeadSize', 0.999999, 'AutoScale', 'off');
% quivZhandle = quiver3(0, 0, 0, 0, 0, 1,  'b', 'ShowArrowHead', 'off', 'MaxHeadSize', 0.999999, 'AutoScale', 'off');
% 
% LimitRatio=1;
% AxisLength=0.1;
% Xlim = [-AxisLength AxisLength] * LimitRatio;
% Ylim = [-AxisLength AxisLength] * LimitRatio;
% Zlim = [-AxisLength AxisLength] * LimitRatio;
% set(gca, 'Xlim', Xlim, 'Ylim', Ylim, 'Zlim', Zlim);
% view(View(1, :));
time=0;
p = plot(linVel(:,3));

p.YDataSource = 'linVel(:,3)';

tic;
while(toc < stopTimer)
    [accel, gyro, mag, timeStamp] = read(imu);
    
    rotators = FUSE(accel,gyro);
    for j = numel(rotators)
        if (i==2)
            time=timeStamp;
        else
            time=[time;timeStamp];
        end
        
        R = quat2rotm(rotators(j));
        tcAcc = R * accel(j,:)';
        imuAcc(i,:)=accel(j,:);
        imuGyro(i,:)=gyro(j,:);

        linAcc(i,:) = tcAcc' - [0 0 9.8];
        linVel(i,:) = linVel(i-1,:) + linAcc(i,:) * samplePeriod;
        linPos(i,:) = linPos(i-1,:) + linVel(i,:) * samplePeriod;

        i=i+1;
        
        refreshdata
        drawnow
    end
end