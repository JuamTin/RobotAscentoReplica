syms theta theta_p theta_2p v v_p L g beta_v u M m
a = -m^2*L^2*cos(theta)*sin(theta);
b = m*L^2*(m*L*theta_p^2*sin(theta)-beta_v*v);
c = m*L^2*u;
d = m*L^2*(M+m*(1-cos(theta)^2));
eq1 = (a+b+c)/(d)-v_p;
a = (m+M)*m*g*L*sin(theta);
b = -m*L*cos(theta)*(m*L*theta_p^2*sin(theta)-beta_v*v);
c = m*L*cos(theta)*u;
eq2 = (a+b+c)/d-theta_2p;

u_desp = solve(eq1 == 0, u);


dinamica = subs(eq2,u,u_desp);
dinamica = subs(dinamica,theta,pi-theta);
dinamica = simplify(subs(dinamica,theta_2p,0));

estable = subs(dinamica,[theta_p v_p], [0 0]);
[v_desp, parameters, conditions] = solve(estable == 0, v,'Real',true,'ReturnConditions',true);
% vpa(subs(v_desp,[theta, m, M, L, g, beta_v],[1*pi/180,1,0.1,25e-2,9.8,0.2]))/7.5e-2