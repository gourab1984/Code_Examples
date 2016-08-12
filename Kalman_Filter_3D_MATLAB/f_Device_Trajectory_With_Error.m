%% Device Trajectory With Added Error
function [x_n,a_L_meas_n,acc_noise] = f_Device_Trajectory_With_Error(n,x)

% Calculation after Adding Error
load('imudata.txt');

% Scale Factor
S_a = [1+.0003 0 0;0 1+.0005 0;0 0 1+.0004]; % scale factor(xyz) [.003 .005 .004]
S_g = [1+.0003 0 0;0 1+.0005 0;0 0 1+.0004]; % scale factor(xyz) [.003 .005 .004]
% Bias
% 25-30 mg converted to m/s^2
b_a = [-0.0002940 0.0001960 -0.0002450]; % Acc Bias
% by averaging gyro data from Ex2
b_g = [0.0001600 -0.0001800 -0.0001500]; % Gyro Bias
% random noise
acc_noise = normrnd(0,0.0001518,66267,3); % N(0,0.1518)
gyro_noise = normrnd(0,0.0001518,66267,3); % N(0,0.3518)
acc_k = 0;
gyro_k = 0;
acc_n = zeros(66267,3);
gyro_n = zeros(66267,3);

switch n
    case 1
        for k = 1:66267
            acc_k=imudata(k,4:6);
            gyro_k=imudata(k,1:3);
            acc_n(k,:) = acc_k+20*acc_noise(k,:);
            gyro_n(k,:) = gyro_k;
            %acc_n(k,:) = acc_k*S_a+b_a+acc_noise(k,:);
            %acc_n(k,:) = acc_k;
            %gyro_n(k1,:) = gyro_k+gyro_noise(k1,:);
            %gyro_n(k1,:) = gyro_k*S_g+b_g+gyro_noise(k1,:);
        end
    case 2
        for k = 1:66267
            acc_k=imudata(k,4:6);
            gyro_k=imudata(k,1:3);
            %acc_n(k,:) = acc_k+acc_noise(k,:);
            gyro_n(k,:) = gyro_k;
            acc_n(k,:) = acc_k*S_a+b_a+acc_noise(k,:);
            %acc_n(k,:) = acc_k;
            %gyro_n(k1,:) = gyro_k+gyro_noise(k1,:);
            %gyro_n(k1,:) = gyro_k*S_g+b_g+gyro_noise(k1,:);
        end
    case 3
        for k = 1:66267
            acc_k=imudata(k,4:6);
            gyro_k=imudata(k,1:3);
            %acc_n(k,:) = acc_k+acc_noise(k,:);
            %gyro_n(k,:) = gyro_k;
            %acc_n(k,:) = acc_k*S_a+b_a+acc_noise(k,:);
            acc_n(k,:) = acc_k;
            gyro_n(k,:) = gyro_k+gyro_noise(k,:);
            %gyro_n(k1,:) = gyro_k*S_g+b_g+gyro_noise(k1,:);
        end
     case 4
        for k = 1:66267
            acc_k=imudata(k,4:6);
            gyro_k=imudata(k,1:3);
            acc_n(k,:) = acc_k+acc_noise(k,:);
            %gyro_n(k,:) = gyro_k;
            %acc_n(k,:) = acc_k*S_a+b_a+acc_noise(k,:);
            %acc_n(k,:) = acc_k;
            gyro_n(k,:) = gyro_k+gyro_noise(k,:);
            %gyro_n(k1,:) = gyro_k*S_g+b_g+gyro_noise(k1,:);
        end
end


% Use gyro data for C update, 3D gyro samples times delta-t
dt = 1/100;
gyro_data1_n = gyro_n;
P_n = (gyro_data1_n.*dt);   % gyro data for C update, 3D gyro samples times delta-t
accelerometer_data1_n=acc_n;
a_B_n = accelerometer_data1_n;
I3 =  eye(3);
p_mod_n = zeros(66267,1);
p_cross_n = cell(66267,1);
g = [0 0 -9.8219].';
C_BtoL_n = cell(66267,1);
a_L_meas_n = zeros(3,66267);
x_n = zeros(6,66268);

% Calculating angle of rotation vector
for i = 1:66267
    p_mod_n(i,1) = sqrt(P_n(i,1)^2+P_n(i,2)^2+P_n(i,3)^2);
end

% Calculating skew symmetric matrix (px) of xyz components 
% of rotation vector
for j = 1:66267
    p_cross_n{j,1} = [0 -P_n(j,3) P_n(j,2);P_n(j,3) 0 -P_n(j,1);-P_n(j,2) P_n(j,1) 0];
end

% creating cell of direction cosine matrices (body vs. local)
for k = 1:66267
    C_BtoL_init_n = I3+((sin(p_mod_n(k,1)))/p_mod_n(k,1))*p_cross_n{k,1} ...
            +((1-cos(p_mod_n(k,1)))/(p_mod_n(k,1))^2)*p_cross_n{k,1}*p_cross_n{k,1};
    if (k == 1)
        %initial direction cosine matrix is identity matrix
        C_BtoL_n{k,1} = I3 * C_BtoL_init_n;
    else
        C_BtoL_n{k,1} = C_BtoL_n{(k-1),1} * C_BtoL_init_n;
    end
end

% Calculating true acceleration (simplified) from measured 
% acceleration data and g
for i1 = 1:66267
    a_L_meas_n(:,i1) = C_BtoL_n{i1,1}*(a_B_n(i1,:)).'+g;
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
    x_n(:,i2) = A*x_n(:,i2-1) + B*a_L_meas_n(:,i2-1);
end


figure
%subplot(2,2,1);
plot3(x_n(1,:),x_n(2,:),x_n(3,:));
%axis(aa)
xlabel('X - Axis');
ylabel('Y - Axis');
zlabel('Z - Axis');
title('3D plot of the object movement with Error');
hold on;
plot3(x_n(1,1),x_n(2,1),x_n(3,1), 'ko', x_n(1,end),x_n(2,end),x_n(3,end), 'ro');
hold off;

figure
%subplot(2,2,2);
plot(x_n(1,:),x_n(2,:));
xlabel('X - Axis');
ylabel('Y - Axis');
title('2D plot of the object movement with Error');

figure
%subplot(1,2,2);
plot(x_n(3,:));
xlabel('time');
ylabel('Vertical movement');
title('plot of the object vertical movement with error in time');


posn_error = zeros(66268,1);

for i3 = 1:66268
    
    posn_error(i3,1) = sqrt((x(1,i3)-x_n(1,i3))^2+(x(2,i3)-x_n(2,i3))^2 ...
                        +(x(3,i3)-x_n(3,i3))^2);
    
end

figure
subplot(1,2,1);
plot(posn_error(:,1));
title('Position error growth with time');
xlabel('time (.01s interval per sample)');
ylabel('Position Error (m/s)');

vel_error = zeros(66268,1);
for i4 = 1:66268
    
    vel_error(i4,1) = sqrt((x(4,i4)-x_n(4,i4))^2+(x(5,i4)-x_n(5,i4))^2 ...
                        +(x(6,i4)-x_n(6,i4))^2);
    
end

%figure
subplot(1,2,2);
plot(vel_error(:,1));
title('Velocity error growth with time');
xlabel('time (.01s interval per sample)');
ylabel('Velocity Error (m/s)');


end