% Arduino連接
% a = arduino('COM3', 'Uno', 'Libraries', 'I2C');
% fs = 100; % Sample Rate in Hz
% imu = mpu9250(a,'SampleRate',fs,'OutputFormat','matrix'); 

% 各參數初始化
linAcc=[0 0 0];
linVel=[0 0 0];
linVelHP=[0 0 0];
linPos=[0 0 0];
eInt = [0 0 0];
q = [1 0 0 0]; % [w x y z]
timeRecord = zeros(1,500);
accRecord=zeros(500,3);
gyrRecord=zeros(500,3);

% 濾波器
order = 1;
filtCutOff = 0.1;
samplePeriod = 1/fs;
[b, a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high');
% freqz(b,a);

% 3D figure create and setting
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

lastTime = datetime('now','TimeZone','local','Format','d-MMM-y HH:mm:ss.SSS');
i=2;
while(i <= 500)
    % 讀取感測器數值
    [acc,time] = imu.readAcceleration;
    gyr = imu.readAngularVelocity;
    % 記錄數值
    accRecord(i,:) = acc;
    gyrRecord(i,:)=gyr;
    % DeltaT
    SamplePeriod = seconds(time-lastTime);
    lastTime=time;
    timeRecord(i)=SamplePeriod;
    % Mahony融合濾波 (沒有用磁力校正)
    q = MahonyAHRS(q,gyr,acc,SamplePeriod,eInt);
    
    % Figure顯示
    R = quat2rotm(q);
    ux = R(1,1);
    vx = R(2,1);
    wx = R(3,1);
    uy = R(1,2);
    vy = R(2,2);
    wy = R(3,2);
    uz = R(1,3);
    vz = R(2,3);
    wz = R(3,3);
    set(quivXhandle, 'xdata', 0, 'ydata', 0, 'zdata', 0,'udata', ux, 'vdata', vx, 'wdata', wx);
    set(quivYhandle, 'xdata', 0, 'ydata', 0, 'zdata', 0,'udata', uy, 'vdata', vy, 'wdata', wy);
    set(quivZhandle, 'xdata', 0, 'ydata', 0, 'zdata', 0,'udata', uz, 'vdata', vz, 'wdata', wz);
    drawnow;
    
    i=i+1;
%     for j = numel(rotators)
%         R = quat2rotm(rotators(j));
%         tcAcc = R * accel(j,:)';
%         linAcc(i,:) = tcAcc' - [0 0 9.8];
%         linVel(i,:) = linVel(i-1,:) + linAcc(i,:) * samplePeriod;
%         if (length(linVel)>3)
%             linVelHP = filtfilt(b, a, linVel);
%             linPos(i,:) = linPos(i-1,:) + linVelHP(i,:) * samplePeriod;
%             if (length(linPos)>3)
%                 linPosHP = filtfilt(b, a, linPos);
%                 ox = linPosHP(i,1);
%                 oy = linPosHP(i,2);
%                 oz = linPosHP(i,3);     
%                 ux = R(1,1);
%                 vx = R(2,1);
%                 wx = R(3,1);
%                 uy = R(1,2);
%                 vy = R(2,2);
%                 wy = R(3,2);
%                 uz = R(1,3);
%                 vz = R(2,3);
%                 wz = R(3,3);
%                 set(orgHandle, 'xdata', ox, 'ydata', oy, 'zdata', oz);
%                 set(quivXhandle, 'xdata', ox, 'ydata', oy, 'zdata', oz,'udata', ux, 'vdata', vx, 'wdata', wx);
%                 set(quivYhandle, 'xdata', ox, 'ydata', oy, 'zdata', oz,'udata', uy, 'vdata', vy, 'wdata', wy);
%                 set(quivZhandle, 'xdata', ox, 'ydata', oy, 'zdata', oz,'udata', uz, 'vdata', vz, 'wdata', wz);
% 
%                 axisLimChanged = false;
%                 if((linPosHP(i,1) - AxisLength) < Xlim(1)), Xlim(1) = linPosHP(i,1) - LimitRatio*AxisLength; axisLimChanged = true; end
%                 if((linPosHP(i,2) - AxisLength) < Ylim(1)), Ylim(1) = linPosHP(i,2) - LimitRatio*AxisLength; axisLimChanged = true; end
%                 if((linPosHP(i,3) - AxisLength) < Zlim(1)), Zlim(1) = linPosHP(i,3) - LimitRatio*AxisLength; axisLimChanged = true; end
%                 if((linPosHP(i,1) + AxisLength) > Xlim(2)), Xlim(2) = linPosHP(i,1) + LimitRatio*AxisLength; axisLimChanged = true; end
%                 if((linPosHP(i,2) + AxisLength) > Ylim(2)), Ylim(2) = linPosHP(i,2) + LimitRatio*AxisLength; axisLimChanged = true; end
%                 if((linPosHP(i,3) + AxisLength) > Zlim(2)), Zlim(2) = linPosHP(i,3) + LimitRatio*AxisLength; axisLimChanged = true; end
%                 if(axisLimChanged), set(gca, 'Xlim', Xlim, 'Ylim', Ylim, 'Zlim', Zlim); end
%                 drawnow;
%             end   
%         else
%             linPos(i,:) = [0 0 0];
%         end
%         i=i+1;
%     end
end