%% PWM characteristics
pwmValues = [0.0 0.2 0.4 0.6 0.8 1];
voltages =[ 0.0 4.88 9.10 10.63 11.40 12.00;
            0.0 4.68 8.96 10.50 11.16 12.00;
            0.0 2.88 7.59  9.65 10.68 12.00]';
        
time = pwmValues;
xLim = [0 1];
accelerations = [voltages(:,1) voltages(:,2) voltages(:,3)];
yLim = [0 12];
displayNames = ["Motor A","Motor B","Motor C"];
axisNames = ["$PWM\;[-]$","$U\;[V]$"];
measurement_case = 'pwm_u';
createfigure(time,accelerations);
%figsave(gcf,measurement_case,15,15);