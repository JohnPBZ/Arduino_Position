% a = arduino('COM3', 'Uno', 'Libraries', 'I2C');
% fs = 100; % Sample Rate in Hz
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
linVelHP=[0 0 0];
linPos=[0 0 0];
i=2;
order = 1;
filtCutOff = 0.1;
[b, a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high');
% freqz(b,a);

fig = figure('NumberTitle', 'off', 'Name', '6DOF Animation');
View = [30 20];
Position = [9 39 1280 720];
set(fig, 'Position', Position);
set(gca, 'drawmode', 'fast');
lighting phong;
set(gcf, 'Renderer', 'zbuffer');
hold on;
axis equal;
grid on;

orgHandle = plot3(0, 0, 0, 'k.');
quivXhandle = quiver3(0, 0, 0, 1, 0, 0,  'r', 'ShowArrowHead', 'off', 'MaxHeadSize', 0.999999, 'AutoScale', 'off');
quivYhandle = quiver3(0, 0, 0, 0, 1, 0,  'g', 'ShowArrowHead', 'off', 'MaxHeadSize', 0.999999, 'AutoScale', 'off');
quivZhandle = quiver3(0, 0, 0, 0, 0, 1,  'b', 'ShowArrowHead', 'off', 'MaxHeadSize', 0.999999, 'AutoScale', 'off');

LimitRatio=1;
AxisLength=0.1;
Xlim = [-AxisLength AxisLength] * LimitRatio;
Ylim = [-AxisLength AxisLength] * LimitRatio;
Zlim = [-AxisLength AxisLength] * LimitRatio;
set(gca, 'Xlim', Xlim, 'Ylim', Ylim, 'Zlim', Zlim);
view(View(1, :));

tic;
while(toc < stopTimer)
    [accel,gyro] = readSensorDataMPU9250(imu);
    rotators = FUSE(accel,gyro);
    for j = numel(rotators)
%         viewer(rotators(j));
        R = quat2rotm(rotators(j));
        tcAcc = R * accel(j,:)';
        linAcc(i,:) = tcAcc' - [0 0 9.8];
        linVel(i,:) = linVel(i-1,:) + linAcc(i,:) * samplePeriod;
        if (length(linVel)>3)
            linVelHP = filtfilt(b, a, linVel);
            linPos(i,:) = linPos(i-1,:) + linVelHP(i,:) * samplePeriod;
            if (length(linPos)>3)
                linPosHP = filtfilt(b, a, linPos);
                ox = linPosHP(i,1);
                oy = linPosHP(i,2);
                oz = linPosHP(i,3);     
                ux = R(1,1);
                vx = R(2,1);
                wx = R(3,1);
                uy = R(1,2);
                vy = R(2,2);
                wy = R(3,2);
                uz = R(1,3);
                vz = R(2,3);
                wz = R(3,3);
                set(orgHandle, 'xdata', ox, 'ydata', oy, 'zdata', oz);
                set(quivXhandle, 'xdata', ox, 'ydata', oy, 'zdata', oz,'udata', ux, 'vdata', vx, 'wdata', wx);
                set(quivYhandle, 'xdata', ox, 'ydata', oy, 'zdata', oz,'udata', uy, 'vdata', vy, 'wdata', wy);
                set(quivZhandle, 'xdata', ox, 'ydata', oy, 'zdata', oz,'udata', uz, 'vdata', vz, 'wdata', wz);

                axisLimChanged = false;
                if((linPosHP(i,1) - AxisLength) < Xlim(1)), Xlim(1) = linPosHP(i,1) - LimitRatio*AxisLength; axisLimChanged = true; end
                if((linPosHP(i,2) - AxisLength) < Ylim(1)), Ylim(1) = linPosHP(i,2) - LimitRatio*AxisLength; axisLimChanged = true; end
                if((linPosHP(i,3) - AxisLength) < Zlim(1)), Zlim(1) = linPosHP(i,3) - LimitRatio*AxisLength; axisLimChanged = true; end
                if((linPosHP(i,1) + AxisLength) > Xlim(2)), Xlim(2) = linPosHP(i,1) + LimitRatio*AxisLength; axisLimChanged = true; end
                if((linPosHP(i,2) + AxisLength) > Ylim(2)), Ylim(2) = linPosHP(i,2) + LimitRatio*AxisLength; axisLimChanged = true; end
                if((linPosHP(i,3) + AxisLength) > Zlim(2)), Zlim(2) = linPosHP(i,3) + LimitRatio*AxisLength; axisLimChanged = true; end
                if(axisLimChanged), set(gca, 'Xlim', Xlim, 'Ylim', Ylim, 'Zlim', Zlim); end
                drawnow;
            end   
        else
            linPos(i,:) = [0 0 0];
        end
        i=i+1;
    end
end