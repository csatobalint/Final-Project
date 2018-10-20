clear all
close all
clc


%Initialize motor parameters
DC_Motor_R = 1.0;
DC_Motor_L = 0;
DC_Motor_K = 0.8;
DC_Motor_J = 0.05;
DC_Motor_B = 0.001;

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
MeasurementShaftSpeed = rpm_to_rads * MeasurementShaftSpeed;
plot(MeasurementTime,MeasurementShaftSpeed);

load('values_02.mat');

