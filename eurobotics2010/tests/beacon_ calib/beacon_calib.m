
%% Data calibration
clear all
load beacon_as_fixed_calib;
offset_cm = 0;                   %-6.2;
first_sample = 2;
x = data(first_sample:end,2).*1.6;               % timer_counts, timer_period = 1.6us
y = data(first_sample:end,1) + offset_cm;   % distance(cm), from 30 cm aprox.

%% Scale and interpolation of data
mu = mean(x);
sigma = std(x);
x_scale = (x-mu)./sigma;
x_scale_inter = linspace(x_scale(1), x_scale(2), 100);
x_inter = linspace(x(1),x(end),100);

%% Fit a polynomial
p = polyfit(x_scale,y,6);
y_fit = polyval(p,x_scale);

%% Results

% compare curves
close all
figure
plot(x,y,'ro')
hold on
plot(x,y_fit)
grid on
legend('Calibration samples', 'Polynomial fit');

% error
error = y-y_fit;
figure
plot(x, error);
legend('error (cm)')

%% Table
% Nota: forma de sacar índice tabla = 832-1- ((table(1,1)-3852)/8)
%x_eval = 6560:-5:2415;
x_eval = 10500:-8:3850;
y_eval = polyval(p,(x_eval-mu)./sigma);

y_int = round(y_eval);
y_inc = y_int(2:end)-y_int(1:end-1);
table = [x_eval; y_int; [0 y_inc]];

figure
stem(table(2,:), table(3,:))

figure
plot(x,y,'ro')
hold on
plot(x_eval,y_eval)
grid on


% %% Inverse fit
% p_inv = polyfit((y-mean(y))./std(y), x, 6);
% y_eval = [30:5:350];
% x_eval = polyval(p_inv, (y_eval-mean(y_eval))./std(y_eval));
% 
% figure
% plot(x_eval,y_eval)
% legend('Pulse(us) = f(dist(cm)')
% 
% table = [x_eval; y_eval]

