%% GPS data error mitigation with Kalman Filater
function x_history_GPS = f_KF_GPS(x,GPSdata)

% Initial state and its covariance matrix (mean square error, MSE) P
x_KF= [0 0 0 0 0 0].';
dt = 1/100;

% Calculating covariance matrix (mean square error, MSE) P
P_GPS = diag(100^2.*ones(6,1));
P_GPS_History = cell(66268,1);
%state transition matrix
Phi = diag(ones(6,1));
% Measurement matrix
H = diag(ones(6,1));

%process noise
Q = diag(20.*ones(6,1));
% Measurement noise
sigmasq_GPS = mean((x.'-GPSdata.').^2);
R_GPS = diag(100^2*sigmasq_GPS);

%Full state history for plotting for plotting
x_history_GPS = zeros(size(x));

%loop  the data
for k = 1:size(x,2)
   % Make Kalman filter propagation
   xk_minus = Phi*x_KF;
   Pk_minus = Phi*P_GPS*(Phi).' + Q;
   K = Pk_minus*H.'/(H*Pk_minus*H.'+ R_GPS);
       
   %Make Kalman filter Measurement update
   y = GPSdata(:,k); %use this as measurement
   x_KF = xk_minus + K*(y - H*xk_minus);
   P_GPS = (eye(6) - K*H) * Pk_minus;
   P_GPS_History{k,1} = P_GPS;
   x_history_GPS(:,k) = x_KF; %put the newest estimate here
       
end

%plotting
figure;
plot3(x_history_GPS(1,:),x_history_GPS(2,:),x_history_GPS(3,:),'.-');
hold on;
plot3(x(1,:), x(2,:), ... 
        x(3,:),'r.-') %+20*ones(size(x(1,:)))
%axis(aa);
title('Kalman Filtered GPS data vs real route (in red)')
hold off;
figure
plot(x_history_GPS(1,:),x_history_GPS(2,:),'.-')
hold on;
plot(x(1,:),x(2,:),'r-');
hold off;
title('Kalman Filtered GPS data 2D vs real route 2D ');

