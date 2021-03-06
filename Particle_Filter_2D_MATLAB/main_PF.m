%%
clear;
close all;
clc;
% checking
%%
close all;
% Load the gyro data for prediction updates
load('DataEx3T1.mat');

% specifying environment(world) size
global world_size;
world_size = 20;

% Creating landmarks for measurement updates
landmarks = zeros(4,2);

% Dividing the area in 12X12 grids and assigning one landmark 
% at every intersection of the grids 
Landmarks_array = zeros(12,12,2);
for i = 1:12
    for j = 1:12
        Landmarks_array(i,j,1) = -100+(j-1)*20; % landmark x co-ordinate
        Landmarks_array(i,j,2) = 0+(i-1)*20; % landmark y co-ordinate
        %cntr = cntr+1;
    end
end

% For simplicity, disregarding accelerometer values and instead assuming
% a constnt velocity for the Robot: 2 m/sec
v = 2;

% sampling interval in seconds (sampling rate = 100Hz)
dt = 0.01;

myrobot = Robot(); % Creating Robot instance from Robot class
empty_robot = Robot(); % Creating empty objects of Robot class
N = 500; % number of particles

T = 200; % number of iterations before plotting and choosing new set of landmarks
T_tot = floor(length(gyro(:,1))/T); % Total number of gyro data

% Adding simulated white noise to gyro
gyro_noise = normrnd(0,0.3518,43560,3); % white noise
gyro_n = gyro + gyro_noise; % gyro data with noise

% Creating empty particle instances
p(1,N) = empty_robot;
p2(1,N) = empty_robot;
p3(1,N) = empty_robot;

% x and y coordinates of Robot path for every T number of iterations
myrobot_history = zeros(T,2);

% average error of Robot path (true vs calculated)
% for every T number of iterations
avg_err = zeros(1,T);

w = zeros(1,N); % particle weight
p_history = zeros(T,N,2); % x and y coordinates history of particles
est_pos = zeros(T,3); % estimated position of the Robot from particles

g_counter = 1;
h1 = figure(1); % figure 1 handle
a1 = axes; % figure 1 axis
h2 = figure(2); % figure 2 handle
a2 = axes; % figure 2 axis

% axis ([-80 120 0 220]); % for plotting with  the whole world view

% Creating Particles
for i = 1:N
    r = Robot();
    r.set(rand * world_size, rand * world_size, rand * 2.0 * pi);
    r.set_noise(0.05, 0.05, 5.0);
    p(i) = r;
end

figure(h2);
title('Average error growth with time');
h2.Name = 'Average error growth with time';
figure(h1);
title('True Path in Blue, estimated path in Red');
hold (a1,'on'),hold (a2,'on');
% itearting for every sample values
for t_tot = 1:T_tot
    %cla
    figure(h1);
    
    % choosing the nearest landmarks of current robot position
    elmnts=(Landmarks_array(:,:,1)>=(myrobot_history(T,1)-20) ... 
            &Landmarks_array(:,:,1)<=(myrobot_history(T,1)+20)) ...
            &(Landmarks_array(:,:,2)>=(myrobot_history(T,2)-20) ... 
            &Landmarks_array(:,:,2)<=(myrobot_history(T,2)+20));
    x1 = Landmarks_array(elmnts); % x co-ordinates of the nearest landmarks
    y1 = Landmarks_array(:,:,2);
    y2 = y1(elmnts); % y co-ordinates of the nearest landmarks
    
    % Landmarks array for the next 200 iteration cycle
    Landmarks_new = [x1 y2];
    landmarks = Landmarks_new(1:4,:);
    
    % iteration cycle for 200 iteraions
    for t = 1:T
        % Robot true movement using gyro data without error
        myrobot = myrobot.move(gyro(g_counter,3), v, dt);
        myrobot_history(t,:) = [(myrobot.x),(myrobot.y)];
        
        % sensing the robot positon based on landmarks
        Z = myrobot.sense(landmarks);

        % Particle movement using noisy gyro data
        for i = 1:N
            p2(i) = (p(i).move(gyro_n(g_counter,3), v, dt));
        end
        p = p2; % storing all the particles

        % Assigning weights to particle based on sense measurements taknen
        % from landmarks
        for i = 1:N
            w(i) = (p(i).measurement_prob(Z,landmarks));
        end

        % Resampling particles based on weight
        index = randi(N); %random index number from the no. of particles
        beta = 0.0;
        mw = max(w); % maximum weight
        for i = 1:N
            beta = beta + rand*2*mw;
            while (beta > w(index))
                beta = beta - w(index);
                index = mod((index + 1) ,N)+1;
            end
            p3(i) = p(index);
            % x,y coordinates of particle history for the resampled particles
            p_history(t,i,:) = [p3(i).x, p3(i).y];
        end
        p = p3; % storing the resampled particles

        % Estimating position from particles
        est_pos(t,:) = f_get_position(p);
        % Estimating position error compared to true positions
        avg_err(t) = f_eval_err(myrobot, p);
        
        % plotting the particles with alternating colors
        if mod(t,2) > 0
            %mod(round(t/10),2) > 0
           plot(a1,p_history(t,:,1),p_history(t,:,2),'y.');
        else
           plot(a1,p_history(t,:,1),p_history(t,:,2),'c.');
        end
        
        % Plotting true Robot movement
        plot(a1,myrobot_history(t,1),myrobot_history(t,2),'bo-');
        
        % Plotting estimated Robot movement based on particles
        plot(a1,est_pos(t,1),est_pos(t,2),'ro-');
        drawnow
        % deleting previous particle plots to avoid clutter
        if mod(t,10) == 0
            delete(findobj(h1,'Color','y'))
            delete(findobj(h1,'Color','c'))
        end
        g_counter = g_counter+1;
    end
    % Plotting true Robot movement
    plot(a1,myrobot_history(1:10:end,1),myrobot_history(1:10:end,2),'bo-');
    % Plotting estimated Robot movement based on particles
    plot(a1,est_pos(1:10:end,1),est_pos(1:10:end,2),'ro-');
    t1 = (g_counter-T+1):g_counter;
    figure(h2);
    % Plotting average error between true position and estimated
    % position
    plot(a2,t1,avg_err,'b');
    pause(3)
    
end
hold (a1,'off'),hold (a2,'off');