%% Device Trajectory Without Error
function x = f_Device_Trajectory_Without_Error()

load('imudata.txt');
% accelerometer_data=imudata(:,4:6); % specific forces in m/s^2
% gyro_data=imudata(:,1:3); % rotation rates in rad/s

% Use gyro data for C update, 3D gyro samples times delta-t
dt = 1/100;
gyro_data1 = imudata(:,1:3);
P = (gyro_data1.*dt);   % gyro data for C update, 3D gyro samples times delta-t
accelerometer_data1=imudata(:,4:6);
a_B = accelerometer_data1;
I3 =  eye(3);
p_mod = zeros(66267,1);
p_cross = cell(66267,1);
g = [0 0 -9.8219].';
C_BtoL = cell(66267,1);
a_L_meas = zeros(3,66267);
x = zeros(6,66268);

% Calculating angle of rotation vector
for i = 1:66267
    p_mod(i,1) = sqrt(P(i,1)^2+P(i,2)^2+P(i,3)^2);
end

% Calculating skew symmetric matrix (px) of xyz components 
% of rotation vector
for j = 1:66267
    p_cross{j,1} = [0 -P(j,3) P(j,2);P(j,3) 0 -P(j,1);-P(j,2) P(j,1) 0];
end

% creating cell of direction cosine matrices (body vs. local)
for k = 1:66267
    C_BtoL_init = I3+((sin(p_mod(k,1)))/p_mod(k,1))*p_cross{k,1} ...
            +((1-cos(p_mod(k,1)))/(p_mod(k,1))^2)*p_cross{k,1}*p_cross{k,1};
    if (k == 1)
        %initial direction cosine matrix is identity matrix
        C_BtoL{k,1} = I3 * C_BtoL_init;
    else
        C_BtoL{k,1} = C_BtoL{(k-1),1} * C_BtoL_init;
    end
end

% Calculating true acceleration (simplified) from measured 
% acceleration data and g
for i1 = 1:66267
    a_L_meas(:,i1) = C_BtoL{i1,1}*(a_B(i1,:)).'+g;
end

% for position and velocity update
A = [1 0 0 dt 0 0
     0 1 0 0 dt 0
     0 0 1 0 0 dt
     0 0 0 1 0 0
     0 0 0 0 1 0
     0 0 0 0 0 1];
 % for acceleration update
 B = [dt^2 0 0
      0 dt^2 0
      0 0 dt^2
      dt 0 0
      0 dt 0
      0 0 dt];
  
% Final calculation of position and velocity at sampled points
for i2 = 2:66268
    % initial position and velocity values would remain 0s
    x(:,i2) = A*x(:,i2-1) + B*a_L_meas(:,i2-1);
end

figure
%subplot(1,2,1);
plot3(x(1,:),x(2,:),x(3,:));
xlabel('X - Axis');
ylabel('Y - Axis');
zlabel('Z - Axis');
title('3D plot of the object movement with circle at start and end');
aa = axis; %for plotting porposes
hold on;
plot3(x(1,1),x(2,1),x(3,1), 'ko', x(1,end),x(2,end),x(3,end), 'ro');
hold off;
grid on;

figure
%subplot(1,2,2);
plot(x(1,:),x(2,:));
xlabel('X - Axis');
ylabel('Y - Axis');
title('2D plot of the object movement in X and Y axis');

figure
%subplot(1,2,2);
plot(x(3,:));
xlabel('time');
ylabel('vertical movement');
title('plot of the object vertical movement in time');


end