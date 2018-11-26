%% Read and save serial data
measurement_case = '2018_10_20/speed_and_acceleration';

if 0
    % Port reset:
    delete(instrfindall);

    % Serial open
    arduino=serial('COM5','BaudRate',57600);

    % Start to read arduino's values and write the result in results.txt
    fopen(arduino)
    fid = fopen(measurement_case,'wt');
    sampling_time = 25;
    measurement_time = 2000;
    for i=1:measurement_time/sampling_time
        y = fscanf(arduino,'%s');
        fprintf(fid,'%s\n',y);
        measurement_time/sampling_time-i
    end
    fclose(fid);
    fclose(arduino);
end

rowOffset = 3;
data = dlmread(measurement_case,',',rowOffset,0)
time = data(:,1)-1000;
xLim = [0 measurement_time];
accelerations = [data(:,2) data(:,3) data(:,4)];
yLim = [0 120];
displayNames = ["Motor A","Motor B","Motor C"];
axisNames = ["$t\;[ms]$","$n\;[rpm]$"];
currentFigure = createfigure3(time,accelerations,axisNames,displayNames,xLim,yLim);
figsave(currentFigure,measurement_case,15,15);

%% Three motors max speed and acceleration
measurement_case = '2018_10_31/max_pwm_forward_10ms_3744';
measurement_time = 2;
h = 0.010;

%angular velocity
a = 1;
b = [1/4 1/4 1/4 1/4];
data = dlmread(strcat(measurement_case,'.txt'),',',0,0);
time = data(:,1)/1000;
xLim = [0 measurement_time];

angular_speeds = abs([filter(b,a,data(:,2)) filter(b,a,data(:,3)) filter(b,a,data(:,4))]);
yLim = [0 1.2*max(max(data(:,[2 3 4])))];
yLim = [0 12];
displayNames = ["Motor A","Motor B","Motor C"];
axisNames = ["$t\;[s]$","$\omega\;[rad/s]$"];
currentFigure=createfigure3(time,angular_speeds,axisNames,displayNames,xLim,yLim);
%figsave(currentFigure,strcat(measurement_case,'_velocity'),15,15);

%angular acceleration
a = 1;
filterFactor = 4;
b=1/filterFactor*ones(1,filterFactor);
angular_acceleration = abs([filter(b,a,data(:,5)) filter(b,a,data(:,6)) filter(b,a,data(:,7))]);
%angular_acceleration =angular_acceleration/(2*pi/3960)/9.549296596425384;
yLim = [0 1.2*max(max(angular_acceleration))];
yLim = [0 100];
displayNames = ["Motor A","Motor B","Motor C"];
axisNames = ["$t\;[s]$","$\varepsilon\;[rad/s^2]$"];
createfigure3(time,angular_acceleration,axisNames,displayNames,xLim,yLim);
%figsave(currentFigure,measurement_case,15,15);

measuredEpsilon=abs(data(:,6));
mTime=data(:,1)/1000;
a = 1;filterFactor = 4;b=1/filterFactor*ones(1,filterFactor);
measuredEpsilonFilter4=abs(filter(b,a,data(:,6)));
a = 1;filterFactor = 10;b=1/filterFactor*ones(1,filterFactor);
measuredEpsilonFilter10=abs(filter(b,a,data(:,6)));
plot(epsilonSimulation(:,1),epsilonSimulation(:,2),epsilonSimulation(:,1),102.568*exp(-8.6799*epsilonSimulation(:,1)),mTime,[measuredEpsilon measuredEpsilonFilter4 measuredEpsilonFilter10]);
epsilonCompare(epsilonSimulation(:,1),[epsilonSimulation(:,2) epsilonSimulationAnal(:,2)],mTime,[measuredEpsilon measuredEpsilonFilter4 measuredEpsilonFilter10]);

epsilonSimulationAnal(:,2)=102.568*exp(-8.6799*epsilonSimulation(:,1));

%Compare acceleration calculations

%simple first order (arduino code)
angular_acceleration_forward = zeros(1,length(data));
for i=2:length(data)
    angular_acceleration_forward(i) = (data(i,2)-data(i-1,2))...
                                    /(time(i)-time(i-1));
end

%central differencing
angular_acceleration_central = zeros(1,length(data));
for i=2:length(data)-1
    angular_acceleration_central(i) = (data(i+1,2)-data(i-1,2))...
                                    /(time(i+1)-time(i-1));
end

%calculate acceleration with higher order discretization
angular_acceleration_5_point = zeros(1,length(data));
for i=3:length(data)-2
    angular_acceleration_5_point(i) = ( -data(i+2,2) + 8*data(i+1,2) - 8*data(i-1,2) + data(i-2,2)   )...
                                    /(12*h);
end

a = 1;
filterFactor = 4;
b=1/filterFactor*ones(1,filterFactor);

hold on
%plot(data(:,1),filter(b,a,data(:,5)))
%plot(data(:,1),data(:,5))
% plot(data(:,1),angular_acceleration_forward(:))
% plot(data(:,1),angular_acceleration_central(:))
% plot(data(:,1),angular_acceleration_5_point(:))
plot(data(:,1),filter(b,a,angular_acceleration_forward(:)))
plot(data(:,1),filter(b,a,angular_acceleration_central(:)))
plot(data(:,1),filter(b,a,angular_acceleration_5_point(:)))
hold off

%% Rolling resistance spinnig
measurement_case = '2018_11_04/12_5_V/spinning_0g';
measurement_time = 2;
h = 0.010;
data = dlmread(strcat(measurement_case,''),',',0,0);
spinning_0 = data(:,2);

a = 1;
filterFactor = 10;
b=1/filterFactor*ones(1,filterFactor);

filtered_0 = filter(b,a,spinning_0(:));
filtered_2167 = filter(b,a,spinning_2167(:));
filtered_3187 = filter(b,a,spinning_3187(:));
filtered_4213 = filter(b,a,spinning_4213(:));

hold on
lastNumber = 490;
plot(filtered_0(end-lastNumber:end,1))
plot(filtered_2167(end-lastNumber:end,1))
plot(filtered_3187(end-lastNumber:end,1))
plot(filtered_4213(end-lastNumber:end,1))
hold off

% hold on
% lastNumber = 20;
% velocities_spinning = [velocity_1720(end-lastNumber:end,1) velocity_2240(end-lastNumber:end,1) velocity_3266(end-lastNumber:end,1) velocity_3744(end-lastNumber:end,1)]
% scaledMeanVelocities = mean(velocities_spinning)./max(mean(velocities_spinning))
% timeData = [1720,2240,3266,3744]-1720
% plot(timeData,scaledMeanVelocities)

hold on
lastNumber = 300;
velocities_spinning = [filtered_0(end-lastNumber:end,1) filtered_2167(end-lastNumber:end,1) filtered_3187(end-lastNumber:end,1) filtered_4213(end-lastNumber:end,1)]
scaledMeanVelocities = mean(velocities_spinning)./max(mean(velocities_spinning))
timeData = [0,2167,3187,4213]
plot(timeData,scaledMeanVelocities)

% lastNumber = 20;
% velocities_forward = [velocity_f_0(end-lastNumber:end,1) velocity_f_1720(end-lastNumber:end,1) velocity_f_2741(end-lastNumber:end,1) velocity_f_3744(end-lastNumber:end,1)];
% plot([0,1720,2741,3744],mean(velocities_forward))













