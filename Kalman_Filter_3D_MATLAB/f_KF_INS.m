%% GNSS INS integration with Kalman Filater
function x_history_INS = f_KF_INS(x,GPSdata,a_L_meas_n,acc_noise,n)

% Initial state and its covariance matrix (mean square error, MSE) P
x_KF= [0 0 0 0 0 0].';
%x_n_KF = zeros(6,66268);
dt = 1/100;

% covariance matrix (mean square error, MSE) P
%P_INS = diag([100^2.*ones(2,1);100^2.*ones(1,1); 100^2.*ones(3,1)]);
%sigmasq_INS = mean((x_history_GPS.'-x_n.').^2);
sigmasq_INS = mean((GPSdata.'-x.').^2);
P_INS = diag(sigmasq_INS); % 100^2.*
% P_INS = diag(100^2.*ones(6,1));
P_INS_History = cell(66268,1);

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

%state transition matrix
%Phi = diag(ones(6,1));
Phi = A;

% Measurement matrix
H = [diag(ones(3,1)) zeros(3,3)];
%H = diag(ones(6,1));

%process noise
w_r = diag(var(acc_noise));
if (n == 1)
    Q = (B*w_r*B.');
else
    Q = (B*w_r*B.')*400^2;
end

% Measurement noise
%sigmasq_INS = mean((x_history_GPS.'-x_n.').^2);
%R_INS = 10^2.*diag(sigmasq_INS(1:3)); % 100^2.*% (1:3)
R_INS = diag([20^2 20^2 20^2]);

%Full state history for plotting
x_history_INS = zeros(size(x));

%loop  the data
for k = 1:size(x,2)
   % Make Kalman filter propagation
   %xk_minus = Phi*x_KF;
    %for i2 = 2:66268
    % % initial position and velocity values would remain 0s
    %x_n(:,i2) = A*x_n(:,i2-1) + B*a_L_meas_n(:,i2-1);
    %end
   if (k == 66268)
       x_history_INS(:,k) = x_history_INS(:,k-1);
   else
       xk_minus = Phi*x_KF + B*a_L_meas_n(:,k);
       Pk_minus = Phi*P_INS*(Phi).' + Q;
       K = Pk_minus*H.'/(H*Pk_minus*H.'+ R_INS);

       %Make Kalman filter Measurement update
       % y = x_history_GPS(1:3,k); % using KF GPS data for measurement %1:3
       y = GPSdata(1:3,k);
       x_KF = xk_minus + K*(y - H*xk_minus);
       P_INS = (eye(6) - K*H) * Pk_minus;
       P_INS_History{k,1} = P_INS;
       x_history_INS(:,k) = x_KF; %put the newest estimate here
   end
          
end

%plotting
figure;
plot3(x_history_INS(1,:),x_history_INS(2,:),x_history_INS(3,:),'b.-');
hold on;
plot3(x(1,:), x(2,:), ... 
        x(3,:),'r.-') %+20*ones(size(x(1,:)))%axis(aa);
%plot3(x_history_GPS(1,:),x_history_GPS(2,:),x_history_GPS(3,:),'g.-');
hold off;
title('Kalman Filtered INS data vs real route (in red)')

figure
plot(x_history_INS(1,:),x_history_INS(2,:),'.-');
hold on;
plot(x(1,:),x(2,:),'r-');
hold off;
title('Kalman Filtered INS data 2D vs real route 2D (in red)');

posn_error_KF = zeros(66268,1);

for i3 = 1:66268
    
    posn_error_KF(i3,1) = sqrt((x(1,i3)-x_history_INS(1,i3))^2 ... 
        +(x(2,i3)-x_history_INS(2,i3))^2+(x(3,i3)-x_history_INS(3,i3))^2);
                        
    
end

figure
subplot(1,2,1);
plot(posn_error_KF(:,1));
title('Position error growth with time');
xlabel('time (.01s interval per sample)');
ylabel('Position Error (m/s)');
grid on;

vel_error_KF = zeros(66268,1);
for i4 = 1:66268
    
    vel_error_KF(i4,1) = sqrt((x(4,i4)-x_history_INS(4,i4))^2 ... 
         +(x(5,i4)-x_history_INS(5,i4))^2+(x(6,i4)-x_history_INS(6,i4))^2);
    
end

%figure
subplot(1,2,2);
plot(vel_error_KF(:,1));
title('Velocity error growth with time');
xlabel('time (.01s interval per sample)');
ylabel('Velocity Error (m/s)');
grid on;
end