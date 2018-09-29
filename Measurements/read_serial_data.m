%% Three motors max speed 

% Port reset:
delete(instrfindall);

% Serial open
arduino=serial('COM5','BaudRate',57600);

% Start to read arduino's values and write the result in results.txt
fopen(arduino)
measurement_case = 'ramp_up_down_controlled';
fid = fopen(measurement_case,'wt');
sampling_time = 200;
measurement_time = 40000;
for i=1:measurement_time/sampling_time
    y = fscanf(arduino,'%s');
    fprintf(fid,'%s\n',y);
    measurement_time/sampling_time-i
end
fclose(fid);
fclose(arduino);

max_speed_directed = dlmread(measurement_case,',',3,0)
dataX = max_speed_directed(:,1)-1000;
xLim = [0 measurement_time];
dataY = [max_speed_directed(:,2) max_speed_directed(:,3) max_speed_directed(:,4) max_speed_directed(:,5)];
yLim = [0 1000];
displayNames = ["Motor A","Motor B","Motor C","target"];
axisNames = ["$t\;[ms]$","$n\;[rpm]$"];
currentFigure = createfigure3(dataX,dataY,axisNames,displayNames,xLim,yLim);
figsave(currentFigure,measurement_case,15,15);
