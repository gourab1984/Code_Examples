%% Simulating GPS data
function GPSdata = f_Simulate_GPS_data(x)
%x_pos = [x(1,:);x(2,:);x(3,:)];
%x_C = [x(:,1:66268);C_BtoL.'];
GPSdata = x+20*randn(size(x)); % 20*
figure,plot3(GPSdata(1,:),GPSdata(2,:),GPSdata(3,:),'.-')
%aa = axis;
hold on;
plot3(x(1,:)+60*ones(size(x(1,:))), x(2,:)+60*ones(size(x(1,:))), ... 
        x(3,:)+60*ones(size(x(1,:))),'r-')
%axis(aa);
title('Simulated GPS data vs real route shifted 20 in xyz')
hold off;

figure,plot(GPSdata(1,:),GPSdata(2,:),'.-');
%aa1 = axis;
hold on;
plot(x(1,:),x(2,:),'r-');
hold off;
title('Simulated GPS data 2D vs real route 2D ');

end