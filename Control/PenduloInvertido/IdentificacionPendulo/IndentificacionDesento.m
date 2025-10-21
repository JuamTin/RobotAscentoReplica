%% obtencion datos
data = importdata("Robotodata.txt");
ts = 0.01;
alfa = data(400:900,1);
angulo = data(400:900,2);
data_sys = iddata(angulo, alfa, ts);


y = angulo;
u = alfa;
% Initial guesses for parameters [b I l K]
initial_params = [0.1 0.1 0.015 1]; %para modelo1


% Lower and upper bounds 
lb = zeros(1,4);
ub = inf*ones(1,4);

% Using fmincon to perform the optimization
options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp');
[params, fval] = fmincon(@(p) objective_function(p, u, y), initial_params, [], [], [], [], lb, ub, [], options);

%comaparacion
[y_est, modelo] = modelos(1,params,u);
sys_est = iddata(y_est,u,ts);
compare(data_sys, sys_est);

function cost = objective_function(params, u, y) 
    %salida de modelo
    [y_est, ~] = modelos(1,params,u); 
    % Compute the cost as the sum of squared errors
    %r^2
    %cost = 1 - sum((datos_observados - datos_predichos).^2) / sum((datos_observados - mean(datos_observados)).^2);
    %rmse
    cost = sqrt(mean((y - y_est).^2));
  
end

function [y_est, model] = modelos(modelo, params,u)
    ts = 0.01;
    t =length(0:length(u(:,1))-1) * ts;
    tf('s');
     %Obteniendo salida para las tf's con los parametros
    m = 0.1; %masa pendulo
    M = 0.6; %masa carro, motores y llantas
    switch(modelo)
        case 1
            b = params(1); %viscosidad (identicacion)
            I = params(2); %Inercia (identificacion)
            l = params(3); %longitud pendulo (identificacion)
            K = params(4); %constate correcion (identificacion)
            q = (M+m)*(I+m*l^2) - (m*l)^2;
            model = K*(m*l*s/q)/(s^3 + (b*(I + m*l^2))*s^2/q - ((M + m)*m*g*l)*s/q - b*m*g*l/q);
            
    end 
    y_est = lsim(model, u, t);
end