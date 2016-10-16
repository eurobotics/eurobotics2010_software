
warning off

p = 100;
i = 0;
d = 0;

cons = 20000;
t = 1;

coef_d = 833391.208;    % pulsos/m
Ts = 0.005;

if(exist('first_run')==0)
    first_run = 1;
    
    s1 = serial('COM1', 'Baudrate', 115200);% 'InputBufferSize', 1000);
    fopen(s1);

    % cs on
    fprintf(s1, 'event cs on');

    % debug cs on
    fprintf(s1, 'log type cs on');
else
    cons = cons + cons;
end

% set pid gains
fprintf(s1, ['gain angle ',num2str(p),' ', num2str(i),' ',num2str(d)]);

% set consign of distance
fprintf(s1, ['consign angle ', num2str(int32(cons))]);

% read values
n = floor(t/Ts);
ret = int32(zeros(n,9));
clear ret;

% flush input
flushinput(s1);
flushinput(s1);

% read values
for(i=1:n)
ret(i,:) = fscanf(s1,'%f: (%d,%d,%d) angle cons= %d fcons= %d err= %d in= %d out= %d');
end

ret = ret(:,5:end);

% plot results
close all
t = [1:n]*Ts;

figure
subplot(3,1,1), plot(t, ret(:,1),'-', t,ret(:,2),t,ret(:,4),'o');
subplot(3,1,2), plot(t, ret(:,3));
subplot(3,1,3), plot(t, ret(:,5));


% close connection
%fclose(s1);
