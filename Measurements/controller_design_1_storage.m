%% Control design

measurement_time = measurement_time/1000;
measurement_case = 'directed_speed_pwm_50';
data = dlmread(measurement_case,',',2,0)
time = (data(:,1)-1000)/1000;
xLim = [0 measurement_time];
accelerations = [data(:,2) data(:,3) data(:,4)];
yLim = [0 120];

%amplitude
A = mean(accelerations(end-50:end,:))
%rise time
A_T = 0.6321 * A;

for i=1:3
    T(i) = interp1q(accelerations(:,i),time,A_T(i))';
end
T
%single storage plant
dataYfitted = A.*(1-exp(-1./T.*time));
measurement_case = 'directed_speed_pwm_50_fitted';
displayNames = ["Motor A","Motor B","Motor C","Motor A (model)","Motor B (model)","Motor C (model)"];
axisNames = ["$t\;[s]$","$n\;[rpm]$"];
currentFigure = createfigure3(time,[accelerations dataYfitted],axisNames,displayNames,xLim,yLim);
figsave(currentFigure,measurement_case,15,15);