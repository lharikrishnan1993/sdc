clear all;
close all;
syms ax ay thetadot del_t v theta x y 

sensor = [[ax];
          [ay];
          [thetadot];
          [sqrt(ax^2 + ay^2)];
          [1]];

state = [[x];
         [y];
         [theta];
         [v];
         [thetadot]];
      
observation_inverse = [[0.5*del_t^2,          0,     0,     0, v*cos(theta)*del_t ];
                       [0,          0.5*del_t^2,     0,     0, v*sin(theta)*del_t ];
                       [0,                    0, del_t,     0,              theta ];
                       [0,                    0,     0, del_t,                  0 ];
                       [0,                    0,     1,     0,                  0 ]];

state_calculated = observation_inverse*sensor;
% subs(state_calculated, [del_t, theta, v, ax, ay, thetadot], [0.01, 0.758, 10, 7.262124030410263, 6.874703962130864, 0])

observation_matrix = inv(observation_inverse)

calculated_sensor = observation_matrix*state_calculated;
subs(observation_matrix, [del_t, theta, v], [0.1, 0, 0])

observation_matrix;