clear all
clc

datos = importdata('MotorDrchTest.txt');

% Acceder a las columnas
pwm = datos(400:end, 1);
speed = -datos(400:end, 2);

% datos = importdata('Martin_validacion.txt');
% 
% % Acceder a las columnas
% pwmV = datos(20:end, 1);
% angleV = datos(20:end, 2);
% speedV = -datos(20:end, 3);


%pwm = medfilt1(pwm, 30);
%speed = medfilt1(speed, 30);
Ts = 1/100; %tiempo muestreo
t = (0:length(pwm)-1)*Ts;
sistema = iddata(speed, pwm, Ts);
plot(sistema)
% Mostrar los datos importados
%graficar_subplots(t,pwm,angle,speed);
% [modelo2, fval1] = Fmincon(pwm, speed);
% 
% load('planta2.mat','modelo1')