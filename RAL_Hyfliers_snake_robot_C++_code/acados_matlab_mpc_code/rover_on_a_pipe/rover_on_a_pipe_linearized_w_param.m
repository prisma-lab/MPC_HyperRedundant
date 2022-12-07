%
% Copyright 2019 Gianluca Frison, Dimitris Kouzoupis, Robin Verschueren,
% Andrea Zanelli, Niels van Duijkeren, Jonathan Frey, Tommaso Sartor,
% Branimir Novoselnik, Rien Quirynen, Rezart Qelibari, Dang Doan,
% Jonas Koenemann, Yutao Chen, Tobias Schöls, Jonas Schlagenhauf, Moritz Diehl
%
% This file is part of acados.
%
% The 2-Clause BSD License
%
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
%
% 1. Redistributions of source code must retain the above copyright notice,
% this list of conditions and the following disclaimer.
%
% 2. Redistributions in binary form must reproduce the above copyright notice,
% this list of conditions and the following disclaimer in the documentation
% and/or other materials provided with the distribution.
%
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% POSSIBILITY OF SUCH DAMAGE.;
%

function model = rover_on_a_pipe_linearized_w_param()

import casadi.*

%% system dimensions
nx = 2;
nu = 3;
np = 11;

%% system parameters
% mu = SX.sym('mu', 1);
mu = 0.85;
m =   11.77204; %10.6+arm_weight;
% L = 0.233674;
Rw = 0.035;
% gamma_s = 1.1456;
% mu0 = tan(gamma_s);
th_s = 0.4252;
Rp = 0.0762;
Ts = 0.01;
g = 9.81;
% th_s_dx = th_s;
% th_s_sx = th_s;
% L1_dx = 1;
% L1_sx = 1;

% np = 1;
% sym_p = mu;

%% named symbolic variables
% states
x1 = SX.sym('x1'); % angle of rod with the vertical [rad]
x2 = SX.sym('x2'); % angular velocity of rod [rad/s]
% L1_dx = SX.sym('L1_dx'); 
% L1_sx = SX.sym('L1_sx'); 
% th_s_dx = SX.sym('th_s_dx'); 
% th_s_sx = SX.sym('th_s_sx'); 
% gamma_s = SX.sym('gamma_s'); 
% L = SX.sym('L');
% x1_0 = SX.sym('x1_0');
% x2_0 = SX.sym('x2_0'); 
% u1_0 = SX.sym('u1_0'); 
% u2_0 = SX.sym('u2_0'); 
% u3_0 = SX.sym('u3_0'); 
% input
F = SX.sym('F');         % horizontal force acting on cart [N]
tau1 = SX.sym('tau1');
tau2 = SX.sym('tau2');
%param
p = SX.sym('p', np, 1); 
L1_dx = p(1); 
L1_sx = p(2); 
th_s_dx = p(3); 
th_s_sx = p(4); 
gamma_s = p(5); 
L = p(6);
x1_0 = p(7);
x2_0 = p(8); 
u1_0 = p(9); 
u2_0 = p(10); 
u3_0 = p(11); 

%% (unnamed) symbolic variables
sym_x = vertcat(x1, x2);
sym_xdot = SX.sym('xdot', nx, 1);
sym_u = vertcat(F, tau1, tau2);

%% dynamics
%expr_f_expl = vertcat(v, ...
%                      dtheta, ...
%                      (- l*m*sin(theta)*dtheta.^2 + F + g*m*cos(theta)*sin(theta))/(M + m - m*cos(theta).^2), ...
%                      (- l*m*cos(theta)*sin(theta)*dtheta.^2 + F*cos(theta) + g*m*sin(theta) + M*g*sin(theta))/(l*(M + m - m*cos(theta).^2)));
sin_x1 = sin(x1_0);
mu0 = tan(gamma_s);
cos_ths = cos(th_s);
cos_ths_dx = cos(th_s_dx);
cos_ths_sx = cos(th_s_sx);
coeff = 1/(m*L*Rw);

A = [0,1;
     g/L*cos(x1_0),0;]; 
B = [0 0 0;
     1/(m*L) coeff*cos(th_s_dx) coeff*cos(th_s_sx)];
 
 lin_sys = A*[x1-x1_0;x2-x2_0]+B*[F-u1_0;tau1-u2_0;tau2-u3_0];

expr_f_expl_0 = vertcat(x2_0, ...
                      ((g*sin_x1/L)+(u1_0/(m*L))+(cos_ths_dx/(m*L*Rw))*u2_0 + (cos_ths_sx/(m*L*Rw))*u3_0) ); 
expr_f_expl = expr_f_expl_0 + lin_sys;
expr_f_impl = expr_f_expl - sym_xdot;                                                                                                   
expr_phi = vertcat(x1+Ts*x2, ...
                      x2+Ts*((g*sin_x1/L)+(F/(m*L))+(cos_ths_dx/(m*L*Rw))*tau1 + (cos_ths_sx/(m*L*Rw))*tau2)); 
%% constraints
expr_h = sym_u;                                             

%% NL constraints
    % Vectors
    u1 = F;
    u2 = tau1;
    u3 = tau2;

    n0 = [sin(x1_0); -cos(x1_0)];
    t0 = [-cos(x1_0); -sin(x1_0)];
    Fp = [-u1_0.*cos(x1_0); -u1_0.*sin(x1_0)];
    Fw1 = [-(u2_0/Rw).*cos(th_s_sx + x1_0); -(u2_0/Rw).*sin(th_s_sx + x1_0)];
    Fw2 = [-(u3_0/Rw).*cos(th_s_dx - x1_0); (u3_0/Rw).*sin(th_s_dx - x1_0)];
    Fg = zeros(length(Fw1),1);
    Fg(length(Fg)/2+1:end) = -m*g;
    Fs = Fp + Fw1 +Fw2 + Fg;

    coeff1 = (mu*m*g*Rw*L1_sx)/(L1_sx*cos(th_s_dx)+L1_dx*cos(th_s_sx));
    coeff2 = (mu*m*g*Rw*L1_dx)/(L1_sx*cos(th_s_dx)+L1_dx*cos(th_s_sx));

    % Inequality Constraints
    h_expr_temp = [u2_0 - coeff1*cos(x1_0);        
         -u2_0 -  coeff1*cos(x1_0);
         u2_0 - coeff2*cos(x1_0);
         -u3_0 -  coeff2*cos(x1_0);          
         -Fs'*n0 + 0.00001;
         Fs'*t0 - mu0*Fs'*n0;
         -Fs'*t0 - mu0*Fs'*n0]';

    coeff3 = (mu*m*g*Rw)/(L1_dx*cos(th_s_sx)+L1_sx*cos(th_s_dx));
    
    % Gradient of the contraints
    diff_u = [0 1 0;
              0 -1 0;
              0 0 1;
              0 0 -1;
              0 -sin(th_s_dx)/Rw sin(th_s_sx)/Rw;
              1 (cos(th_s_dx)-mu0*sin(th_s_dx))/Rw (cos(th_s_sx)+mu0*sin(th_s_sx))/Rw;
              -1 -(cos(th_s_dx)+mu0*sin(th_s_dx))/Rw -(cos(th_s_sx)-mu0*sin(th_s_sx))/Rw];
          
    diff_x = [coeff3*L1_sx*sin(x1_0) 0;
              coeff3*L1_sx*sin(x1_0) 0;
              coeff3*L1_dx*sin(x1_0) 0;
              coeff3*L1_dx*sin(x1_0) 0;
              m*g*sin(x1_0) 0;
              m*g*(cos(x1_0) + mu0*sin(x1_0)) 0;
              -m*g*(cos(x1_0) - mu0*sin(x1_0)) 0];     
     
     h_expr = h_expr_temp + (diff_x*[x1-x1_0;x2-x2_0]+diff_u*[F-u1_0;tau1-u2_0;tau2-u3_0])';
     
%% cost
W_x = diag([0.001 0.001]);                                  % Q 
W_u = diag([0.1 0.1 0.1]);                                  % R
expr_ext_cost_e = sym_x'* W_x * sym_x;                       
expr_ext_cost = expr_ext_cost_e + sym_u' * W_u * sym_u;     
% nonlinear least sqares
cost_expr_y = vertcat(sym_x, sym_u);
W = blkdiag(W_x, W_u);
model.cost_expr_y_e = sym_x;
model.W_e = W_x;


%% populate structure
model.nx = nx;
model.nu = nu;
model.np = np;
model.sym_x = sym_x;
model.sym_xdot = sym_xdot;
model.sym_u = sym_u;
model.sym_p = p;
model.expr_f_expl = expr_f_expl;
model.expr_f_impl = expr_f_impl;
model.expr_phi = expr_phi;
model.expr_h = expr_h;
model.expr_ext_cost = expr_ext_cost;
model.expr_ext_cost_e = expr_ext_cost_e;
model.constr_expr_h = h_expr;
model.cost_expr_y = cost_expr_y;
model.W = W;

end
