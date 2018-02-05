% script to generate useful quantities for the quadrotor
% simulator
global total_mass LK RYaw
total_mass = 0.55;

% System matrices
A = [ 0 0 0 1 0 0 0; ...
      0 0 0 0 1 0 0; ...
      0 0 0 0 0 1 0; ...
        zeros(4,7)  ];
B = [ zeros(3,4); ...
       eye(4) ];
     
% LQR params
Q = [ 10     0     0     0     0     0     0
      0     10     0     0     0     0     0
      0     0     10     0     0     0     0
      0     0     0     0.1    0     0     0
      0     0     0     0     0.1    0     0
      0     0     0     0     0     0.1     0
      0     0     0     0     0      0     1 ];
R = [ 8     0     0     0
      0     8     0     0
      0     0     1     0
      0     0     0     1 ];

% Feedback gain from LQR
LK = lqr( A, B, Q, R );
LKD = lqrd( A, B, Q, R, 1 );

% temporary fix
RYaw = [  cos(0) -sin(0)  0; ...
          sin(0)  cos(0)  0;
          0       0       1 ];