%% Three motors max speed 
measurement_case = 'controlled_100_p_02_i_11';

if 1
    % Port reset:
    delete(instrfindall);

    % Serial open
    arduino=serial('COM5','BaudRate',57600);

    % Start to read arduino's values and write the result in results.txt
    fopen(arduino)
    fid = fopen(measurement_case,'wt');
    sampling_time = 10;
    measurement_time = 5000;
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

measurement_time = 5000/1000;
measurement_case = 'directed_speed_pwm_50';
max_speed_directed = dlmread(measurement_case,',',2,0)
dataX = (max_speed_directed(:,1)-1000)/1000;
xLim = [0 measurement_time];
dataY = [max_speed_directed(:,2) max_speed_directed(:,3) max_speed_directed(:,4)];
yLim = [0 120];

%amplitude
A = mean(dataY(end-50:end,:))
%rise time
A_T = 0.6321 * A;

for i=1:3
    T(i) = interp1q(dataY(:,i),dataX,A_T(i))';
end
T
%single storage plant
dataYfitted = A.*(1-exp(-1./T.*dataX));
measurement_case = 'directed_speed_pwm_50_fitted';
displayNames = ["Motor A","Motor B","Motor C","Motor A (model)","Motor B (model)","Motor C (model)"];
axisNames = ["$t\;[s]$","$n\;[rpm]$"];
currentFigure = createfigure3(dataX,[dataY dataYfitted],axisNames,displayNames,xLim,yLim);
figsave(currentFigure,measurement_case,15,15);

%% PWM characteristics
pwmValues = [0.0 0.2 0.4 0.6 0.8 1];
voltages =[ 0.0 4.88 9.10 10.63 11.40 12.00;
            0.0 4.68 8.96 10.50 11.16 12.00;
            0.0 2.88 7.59  9.65 10.68 12.00]';
        
dataX = pwmValues;
xLim = [0 1];
dataY = [voltages(:,1) voltages(:,2) voltages(:,3)];
yLim = [0 12];
displayNames = ["Motor A","Motor B","Motor C"];
axisNames = ["$PWM\;[-]$","$U\;[V]$"];
measurement_case = 'pwm_u';
createfigure(dataX,dataY);
figsave(gcf,measurement_case,15,15);


%%
zeta = 0.25;
w0 = 3;
H = tf([w0^2 1],[1,2*zeta*w0,w0^2])
stepplot(H)


%% Three motors max speed and acceleration
measurement_case = 'speed_and_acceleration';

if 1
    % Port reset:
    delete(instrfindall);

    % Serial open
    arduino=serial('COM5','BaudRate',57600);

    % Start to read arduino's values and write the result in results.txt
    fopen(arduino)
    fid = fopen(measurement_case,'wt');
    sampling_time = 25;
    measurement_time = 3000;
    for i=1:measurement_time/sampling_time
        y = fscanf(arduino,'%s');
        fprintf(fid,'%s\n',y);
        measurement_time/sampling_time-i
    end
    fclose(fid);
    fclose(arduino);
end


a = 1;
b = [1/4 1/4 1/4 1/4];b=1;
max_speed_directed = dlmread(measurement_case,',',3,0)
dataX = max_speed_directed(:,1)-1000;
xLim = [0 measurement_time];
dataY = [filter(b,a,max_speed_directed(:,5)) filter(b,a,max_speed_directed(:,6)) filter(b,a,max_speed_directed(:,7))];
yLim = [0 1];
displayNames = ["Motor A","Motor B","Motor C"];
axisNames = ["$t\;[ms]$","$n\;[rpm]$"];
currentFigure = createfigure3(dataX,dataY,axisNames,displayNames,xLim,yLim);
figsave(currentFigure,measurement_case,15,15);

%%
t=dataX;
x=max_speed_directed(:,5);

y = filter(b,a,x);

t = 1:length(x);
plot(t,x,'--',t,y,'-')
legend('Original Data','Filtered Data')




