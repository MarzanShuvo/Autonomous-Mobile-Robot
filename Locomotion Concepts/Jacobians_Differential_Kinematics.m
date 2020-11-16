alpha = sym('alpha','real');
beta = sym('beta','real');
gamma = sym('gamma','real');

q = [alpha;beta;gamma];

r_BF_inB = [...
    - sin(beta + gamma) - sin(beta);...
  sin(alpha)*(cos(beta + gamma) + cos(beta) + 1) + 1;...
  -cos(alpha)*(cos(beta + gamma) + cos(beta) + 1)];


% determine the foot point Jacobian J_BF_inB=d(r_BF_inB)/dq
J_BF_inB = [0, -cos(beta+gamma)-cos(beta), -cos(beta+gamma);
           cos(alpha)*(cos(beta + gamma) + cos(beta) + 1), sin(alpha)*(-sin(beta+gamma)-sin(beta)), sin(alpha)*(-sin(beta+gamma));
           sin(alpha)*(cos(beta + gamma) + cos(beta) + 1), -cos(alpha)* (-sin(beta+gamma)-sin(beta)), -cos(alpha)*(-sin(beta+gamma))]

% what generalized velocity dq do you have to apply in a configuration q = [0;60°;-120°]
% to lift the foot in vertical direction with v = [0;0;-1m/s];


dq = pinv(J_BF_inB)*[0,0,-1]';
dq = double(subs(dq, [alpha, beta, gamma], [0,60*pi/180,-120*pi/180]))
