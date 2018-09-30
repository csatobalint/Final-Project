%% Three motors max speed 

if 0
    % Port reset:
    delete(instrfindall);

    % Serial open
    arduino=serial('COM5','BaudRate',57600);

    % Start to read arduino's values and write the result in results.txt
    fopen(arduino)
    measurement_case = 'directed_speed_pwm_30_polling';
    fid = fopen(measurement_case,'wt');
    sampling_time = 10;
    measurement_time = 3000;
    for i=1:measurement_time/sampling_time
        y = fscanf(arduino,'%s');
        fprintf(fid,'%s\n',y);
        measurement_time/sampling_time-i
    end
    fclose(fid);
    fclose(arduino);
end

max_speed_directed = dlmread(measurement_case,',',3,0)
dataX = max_speed_directed(:,1)-1000;
xLim = [0 measurement_time];
dataY = [max_speed_directed(:,2) max_speed_directed(:,3) max_speed_directed(:,4)];
yLim = [0 120];
displayNames = ["Motor A","Motor B","Motor C"];
axisNames = ["$t\;[ms]$","$n\;[rpm]$"];
currentFigure = createfigure3(dataX,dataY,axisNames,displayNames,xLim,yLim);
figsave(currentFigure,measurement_case,15,15);


%% Control design

measurement_time = 2000;
measurement_case = 'directed_speed_pwm_100';
max_speed_directed = dlmread(measurement_case,',',2,0)
dataX = max_speed_directed(:,1)-1000;
xLim = [0 measurement_time];
dataY = [max_speed_directed(:,2) max_speed_directed(:,3) max_speed_directed(:,4)];
yLim = [0 120];

%amplitude
A = mean(dataY(end-50:end,:))
%rise time
A_T = 0.6321 * A
T = interp1q(y,x,A_T')'

%single storage plant
dataYfitted = A.*(1-exp(-1./T.*dataX))

displayNames = ["Motor A","Motor B","Motor C","Motor A (model)","Motor B (model)","Motor C (model)"];
axisNames = ["$t\;[ms]$","$n\;[rpm]$"];
currentFigure = createfigure3(dataX,[dataY dataYfitted],axisNames,displayNames,xLim,yLim);







