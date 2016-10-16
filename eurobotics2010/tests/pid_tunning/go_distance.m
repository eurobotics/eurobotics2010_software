function go_distance(s,vel,dis)

Ts = 0.02;
coef_d = 833391.208;    % pulsos/m
coef_v = 4166.956*Ts/(0.005);      % (pulsos/Ts)/(m/s)
coef_v_ = 4166.956;      % (pulsos/Ts)/(m/s)
coef_a = 20.835*Ts/(0.005^2);        % (pulsos/Ts^2)/(m/s^2)    

% set aceleration and speed maximus on cs
% fprintf(s, ['quadramp distance ',...
%     num2str(uint32(acc*coef_a)),' ',num2str(uint32(acc*coef_a)),' '...
%     num2str(uint32(vel*coef_v)),' ',num2str(uint32(vel*coef_v))])
% 
% fprintf(s, ['quadramp angle ',...
%     num2str(uint32(acc*coef_a)),' ',num2str(uint32(acc*coef_a)),' '...
%     num2str(uint32(vel*coef_v)),' ',num2str(uint32(vel*coef_v))])

% set speed maximus of trajectory
fprintf(s, ['traj_speed distance ', num2str(uint32(vel*coef_v_))])
%fprintf(s, ['traj_speed angle ', num2str(uint32(vel*coef_v))])


% log on
fprintf(s, 'log type cs on')
fprintf(s, 'log level 5')

% calcule samples
log_dis = zeros(10,5);
log_ang = zeros(10,5);
linelog = char([]);

% go distance relative
flushinput(s);
fprintf(s, ['goto d_rel ', num2str(int32(dis))])

% log data
i = 1;
j = 1;
while(1)
    % read line
    linelog = fgetl(s);

    % end trajectory
    [a count] = sscanf(linelog, 'returned %s');
    if(count == 1)
        break;
    end
    
    % distace log
    [a count] = sscanf(linelog, '%*d.%*3d: (%*d,%*d,%*d) distance cons= %d fcons= %d err= %d in= %d out= %d');
    if(count == 5)
        log_dis(i,:) = a;
        i=i+1;
    end

    % angle log
    [a count] = sscanf(linelog, '%*d.%*3d: (%*d,%*d,%*d) angle cons= %d fcons= %d err= %d in= %d out= %d');
    if(count == 5)
        log_ang(j,:) = a;
        j=j+1;
    end    
end

% off logs
fprintf(s, 'log type cs off');

% plot results
close all
n = 1;
n = min(length(log_dis), length(log_ang));
log_dis = log_dis(1:n,:);
log_ang = log_ang(1:n,:);
t = [1:n]*Ts;

figure
subplot(3,2,1), plot(t, log_dis(:,1),'-b', t,log_dis(:,2),t,log_dis(:,4),'-r');
legend('cons', 'fcons', 'pos dis')
subplot(3,2,2), plot(t, log_ang(:,1),'-b', t,log_ang(:,2),t,log_ang(:,4),'-r');
legend('cons', 'fcons', 'pos ang')


subplot(3,2,3), plot(t, log_dis(:,3));
legend('error')
subplot(3,2,4), plot(t, log_ang(:,3));
legend('error')

subplot(3,2,5), plot(t, log_dis(:,5));
legend('out cs_dist')

subplot(3,2,6), plot(t, log_ang(:,5));
legend('out cs_ang')


% distance, speed and aceceleration
dis_pos_cons = log_dis(:,2);
dis_v_cons = dis_pos_cons(2:end)-dis_pos_cons(1:end-1);
dis_a_cons = dis_v_cons(2:end)-dis_v_cons(1:end-1);

dis_pos = log_dis(:,4);
dis_v = dis_pos(2:end)-dis_pos(1:end-1);
dis_a = dis_v(2:end)-dis_v(1:end-1);

ang_pos_cons = log_ang(:,2);
ang_v_cons = ang_pos_cons(2:end)-ang_pos_cons(1:end-1);
ang_a_cons = ang_v_cons(2:end)-ang_v_cons(1:end-1);

ang_pos = log_ang(:,4);
ang_v = ang_pos(2:end)-ang_pos(1:end-1);
ang_a = ang_v(2:end)-ang_v(1:end-1);

figure
subplot(3,2,1), plot(t, dis_pos_cons./coef_d, t, dis_pos./coef_d);
legend('dis_cons(m)', 'dis(m)')
subplot(3,2,2), plot(t, ang_pos_cons./coef_d, t, ang_pos./coef_d);
legend('ang_cons(deg))', 'ang(deg)')

subplot(3,2,3), plot(t(1:end-1), dis_v_cons./coef_v, t(1:end-1), dis_v./coef_v);
legend('dis_cons(m/s)', 'dis(m/s)')
subplot(3,2,4), plot(t(1:end-1), ang_v_cons./coef_v, t(1:end-1), ang_v./coef_v);
legend('ang_cons(deg/s)', 'ang(deg/s)')

subplot(3,2,5), plot(t(1:end-2), dis_a_cons./coef_a, t(1:end-2), dis_a./coef_a);
legend('dis_cons(m/s^2)', 'dis(m/s^2)')
subplot(3,2,6), plot(t(1:end-2), ang_a_cons./coef_a, t(1:end-2), ang_a./coef_a);
legend('ang_cons(deg/s^2)', 'ang(deg/s^2)')

