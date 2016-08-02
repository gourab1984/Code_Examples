%%
clear;
close all;
clc;
% checking
%%
close all;
load('DataEx3T1.mat');
% global landmarks;
global world_size;
world_size = 20;
landmarks = zeros(4,2);
Landmarks_array = zeros(12,12,2);
for i = 1:12
    for j = 1:12
        Landmarks_array(i,j,1) = -100+(j-1)*20;
        Landmarks_array(i,j,2) = 0+(i-1)*20;
        %cntr = cntr+1;
    end
end
v = 2; % assuming constant velocity for object
dt = 0.01; % sampling iterval in seconds (sampling rate = 100Hz)

myrobot = Robot(); % Creating device instance
empty_robot = Robot(); % Creating empty objects
N = 500;

T = 200;
T_tot = floor(length(gyro(:,1))/T);

% Adding white noise to gyro
gyro_noise = normrnd(0,0.3518,43560,3);
gyro_n = gyro + gyro_noise;

p(1,N) = empty_robot;
p2(1,N) = empty_robot;
p3(1,N) = empty_robot;

myrobot_history = zeros(T,2);
avg_err = zeros(1,T);
w = zeros(1,N);
p_history = zeros(T,N,2);
est_pos = zeros(T,3);

g_counter = 1;
h1 = figure(1);
a1 = axes;
h2 = figure(2);
a2 = axes;
% axis ([-80 120 0 220]);

% Creating Particles
for i = 1:N
    r = Robot();
    r.set(rand * world_size, rand * world_size, rand * 2.0 * pi);
    r.set_noise(0.05, 0.05, 5.0); % (0.05, 0.05, 5.0);
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
    elmnts=(Landmarks_array(:,:,1)>=(myrobot_history(T,1)-20) ... 
            &Landmarks_array(:,:,1)<=(myrobot_history(T,1)+20)) ...
            &(Landmarks_array(:,:,2)>=(myrobot_history(T,2)-20) ... 
            &Landmarks_array(:,:,2)<=(myrobot_history(T,2)+20));
    x1 = Landmarks_array(elmnts);
    y1 = Landmarks_array(:,:,2);
    y2 = y1(elmnts);
    Landmarks_new = [x1 y2];
    landmarks = Landmarks_new(1:4,:);
    %plot(a1,landmarks(:,1),landmarks(:,2),'g*');
    for t = 1:T
        % Device true movement using gyro data without error
        myrobot = myrobot.move(gyro(g_counter,3), v, dt);
        myrobot_history(t,:) = [(myrobot.x),(myrobot.y)];
        Z = myrobot.sense(landmarks);

        % Particel movement using noisy gyro data
        for i = 1:N
            p2(i) = (p(i).move(gyro_n(g_counter,3), v, dt));
        end
        p = p2;

        % Assigning weights to particle based on measurements taknen from
        % landmarks
        for i = 1:N
            w(i) = (p(i).measurement_prob(Z,landmarks));
        end

        index = randi(N);
        beta = 0.0;
        mw = max(w);
        % Resampling particles based on weight
        for i = 1:N
            beta = beta + rand*2*mw;
            while (beta > w(index))
                beta = beta - w(index);
                index = mod((index + 1) ,N)+1;
            end
            p3(i) = p(index);
            p_history(t,i,:) = [p3(i).x, p3(i).y];
        end
        p = p3;

        % Estimating position from particles
        est_pos(t,:) = f_get_position(p);
        % Estimating position error compared to true positions
        avg_err(t) = f_eval_err(myrobot, p);
        % plotting the particles
        %if mod(t,10) == 1
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
        %end
        drawnow
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
    %delete(findobj(h1,'Color','g'))
    %close(2);
end
hold (a1,'off'),hold (a2,'off');