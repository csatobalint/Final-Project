%% Forward and backward
measurement_case = 'forward_backward.txt';
measurement_time = 8;

data = dlmread(measurement_case,',',3,0);
time = (data(:,1)-0)/1000;
xLim = [0 measurement_time];

a = 1;
b = [1/4 1/4 1/4 1/4];b=1;
velocities = [filter(b,a,data(:,2)) filter(b,a,data(:,3)) filter(b,a,data(:,4))];
yLim = [-120 120];
displayNames = ["Motor A","Motor B","Motor C"];
axisNames = ["$t\;[ms]$","$n'\;[rpm/s]$"];
currentFigure = createfigure3(time,velocities,axisNames,displayNames,xLim,yLim);
%figsave(currentFigure,measurement_case,15,15);

MeasurementTime = time;
MeasurementShaftSpeed = (filter(b,a,data(:,2))+filter(b,a,data(:,3))+filter(b,a,data(:,4)))/3;

a = 1;
b = [1/4 1/4 1/4 1/4];
accelerations = [filter(b,a,data(:,5)) filter(b,a,data(:,6)) filter(b,a,data(:,7))];
yLim = [-5 5];
displayNames = ["Motor A","Motor B","Motor C"];
axisNames = ["$t\;[ms]$","$n'\;[rpm/s]$"];
currentFigure = createfigure3(time,accelerations,axisNames,displayNames,xLim,yLim);
%figsave(currentFigure,measurement_case,15,15);


%% Parameter estimation validation
measurement_case = '2018_10_31/parameter_estimation_validation_50_75_100.txt';
measurement_time = 20;

data = dlmread(measurement_case,',',1,0);
time = (data(:,1)-0)/1000;
xLim = [0 measurement_time];

a = 1;
b = [1/4 1/4 1/4 1/4];
velocities = [filter(b,a,data(:,2)) filter(b,a,data(:,3)) filter(b,a,data(:,4))];
yLim = [-1 15];
displayNames = ["Motor A","Motor B","Motor C"];
axisNames = ["$t\;[ms]$","$n'\;[rpm/s]$"];
currentFigure = createfigure3(time,velocities,axisNames,displayNames,xLim,yLim);
%figsave(currentFigure,measurement_case,15,15);

MeasurementTime = time;
MeasurementShaftSpeed = (filter(b,a,data(:,2))+filter(b,a,data(:,3)))/2;

validation_figure(tout,yout,MeasurementTime,MeasurementShaftSpeed)
figsave(gcf,'validation_figure_current',15,15);