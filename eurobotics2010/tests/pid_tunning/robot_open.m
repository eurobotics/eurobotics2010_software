function s = robot_open(port)
%
% Open serial connection with the robot
% Example:
% s = robot_open('COM1')
%
s = serial(port, 'Baudrate', 115200, 'InputBufferSize', 1000);
fopen(s);
