%function [p_ref p_out v a vmot] = cs_distance_plot(acc,vel,dis)

warning off

acc = 3;
vel = 0.8;
dis = 1;
t = 1.5;

coef_d = 833391.208;    % pulsos/m
coef_v = 4166.956;      % (pulsos/Ts)/(m/s)
coef_a = 20.835;        % (pulsos/Ts^2)/(m/s^2)    
Ts = 0.005;

%s1 = serial('COM1', 'Baudrate', 115200);% 'InputBufferSize', 1000);
%fopen(s1);

% set aceleration and speed maximus
fprintf(s1, ['quadramp distance ',...
    num2str(uint32(acc*coef_a)),' ',num2str(uint32(acc*coef_a)),' '...
    num2str(uint32(vel*coef_v)),' ',num2str(uint32(vel*coef_v))]);

% cs on
fprintf(s1, 'event cs on');

% set consign of distance
fprintf(s1, ['consign distance ', num2str(int32(dis*coef_d))]);

% read values
n = floor(t/Ts);
ret = int32(zeros(n,5));
clear ret;

% flush input
flushinput(s1);

% read values
for(i=1:1:n)
ret(i,:) = fscanf(s1,'distance cons= %d fcons= %d err= %d in= %d out= %d');
end

% plot results
close all
t = [1:n]*Ts;

figure
subplot(3,1,1), plot(t, ret(:,1),'-', t,ret(:,2),t,ret(:,4),'o');
subplot(3,1,2), plot(t, ret(:,3));
subplot(3,1,3), plot(t, ret(:,5));

% distance, speed and aceceleration
d = ret(:,4);
v = d(2:end)-d(1:end-1);
a = v(2:end)-v(1:end-1);

figure
subplot(3,1,1), plot(t, d./coef_d);
subplot(3,1,2), plot(t(1:end-1), v./coef_v);
subplot(3,1,3), plot(t(1:end-2), a./coef_a);

% close connection
%fclose(s1);
