%% Cargando datos
%Train
Ts = 0.01;
datos_train = importdata('MotorDrchTrain.txt');
pwmTrain = datos_train(400:end, 1)*10/100;
speedTrain = datos_train(400:end, 2);
real_train = iddata(speedTrain, pwmTrain, Ts);

%Test
datos_test = importdata('MotorDrchTest.txt');
pwmTest = datos_test(400:end, 1)*10/100;
speedTest = -datos_test(400:end, 2);
real_test = iddata(speedTest, pwmTest, Ts);
%% Identificacion
% Initial guesses for parameters [K J b e]
initial_params = [1.837 4.9857 4.4758 0.0205]; 

% Lower and upper bounds 
n = length(initial_params);
lb = zeros(1,n);
ub = inf*ones(1,n);

% Using fmincon to perform the optimization
options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp');
[params, fval] = fmincon(@(p) objective_function(p, pwmTrain, speedTest), initial_params, [], [], [], [], lb, ub, [], options);

%% opteniendo modelo
[modelo, y_model] =  eval_model(params, pwmTrain);
y_modelTr = iddata(y_model, pwmTrain, Ts);
plot(y_model)
figure(1)
compare(real_train, y_modelTr);
figure(2)
[modelo, y_model] =  eval_model(params, pwmTest);
y_modelTs = iddata(y_model, pwmTest, Ts);
compare(real_test, y_modelTs);

%% funciones
function cost = objective_function(p,u,y)
    [model, y_model] = eval_model(p,u);
    %cost = sqrt(mean((y - y_model).^2)); %RSME
    %cost = sum((y - y_model).^2) / sum((y - mean(y)).^2); %R2
    cost = sum(abs(y-y_model))/length(u); %MAE
    %cost = sum(abs(y-y_model))/sum(y); %MAPE
end

function [model, y_model] = eval_model(p,u)
    s = tf('s');
    Ts = 0.01;
    t = (0:length(u)-1)*Ts;
    K = p(1);
    J = p(2);
    b = p(3);
    e = p(4);
    
    model = K/((J*s + b)*(e) + K^2);
    y_model = lsim(model, u', t);
end