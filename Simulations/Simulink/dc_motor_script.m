clear all
close all
clc


%Initialize motor parameters
DC_Motor_R = 10;
DC_Motor_L = 0.01;
DC_Motor_K = 1;
DC_Motor_J = 0.001;
DC_Motor_B = 0.01;

%Initialize motor parameters
R = 10;
L = 0.01;
K = 1;
J = 0.001;
B = 0.01;

%Reverse Port Signal
initialTime = 0.001; finalTime = 8; timeStep = 0.001; N=finalTime/timeStep;
ReversePortTime = linspace(initialTime,finalTime,N)';
ReversePortVoltage = [zeros(1,N/4) 5*ones(1,N/4) zeros(1,N/4) 5*ones(1,N/4)]';
%ReversePortVoltage = [zeros(1,N/4) zeros(1,N/4) zeros(1,N/4) zeros(1,N/4)]';

figure(1)
plot(ReversePortTime,ReversePortVoltage);

figure(2)
load('ShaftSpeedRPM.mat');
rpm_to_rads = 0.104719755;
%MeasurementShaftSpeed = rpm_to_rads * MeasurementShaftSpeed;
plot(MeasurementTime,MeasurementShaftSpeed);

load('values_03.mat');

plot(tout,i_a)

%% Validation
clc
clear all
close all

load('ShaftSpeedValidationOLD.mat');
load('values_03.mat');
load('ShaftSpeedValidation.mat');
rpm_to_rads = 0.104719755;
oldMaxSpeed = rpm_to_rads*mean(MeasurementShaftSpeedOLD([160:190],1));
newMaxSpeed = mean(MeasurementShaftSpeed([1400:1450],1));
scaleShaftSpeed = oldMaxSpeed/newMaxSpeed;
%MeasurementShaftSpeed = scaleShaftSpeed * MeasurementShaftSpeed;

figure(1)
plot(MeasurementTime,MeasurementShaftSpeed);

figure(2)
%Reverse Port Signal
initialTime = 0.001; finalTime = 20; timeStep = 0.001; N=finalTime/timeStep;
InputVoltageTime = linspace(initialTime,finalTime,N)';
InputVoltageSignal = 5/12*[9.60*ones(1,5*N/20) 10.98*ones(1,5*N/20) 12*ones(1,5*N/20) zeros(1,5*N/20)]';
plot(InputVoltageTime,InputVoltageSignal);



