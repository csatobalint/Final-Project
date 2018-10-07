motor_B = 0.5;
motor_J=0.01;
motor_K=0.02;
motor_L=0.01;
motor_R=3;

measurement_case = 'directed_speed_pwm_100';
max_speed_directed = dlmread(measurement_case,',',2,0)

measurment_time = (max_speed_directed(:,1)-1000)/1000;
measurment_voltage = 12*ones(length(measurment_time),1);
measurment_speed = max_speed_directed(:,2);

plot(measurment_time,measurment_speed)