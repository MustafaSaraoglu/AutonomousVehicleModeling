syms t a0 a1 a2 a3 a4 a5;

%           a0  a1   a2    a3     a4       a5
d =         [1  t   t^2   t^3    t^4    t^5];
d_dot =     [0  1   2*t   3*t^2  4*t^3  5*t^4];
d_ddot =    [0  0   2     6*t    12*t^2  20*t^3];

t_i = 0;

d_i =         [1  t_i   t_i^2   t_i^3    t_i^4    t_i^5]; % -2.5
d_dot_i =     [0  1   2*t_i   3*t_i^2  4*t_i^3  5*t_i^4]; %  0
d_ddot_i =    [0  0   2     6*t_i    12*t_i^2  20*t_i^3]; %  0

t_f = 10;

d_f =         [1  t_f   t_f^2   t_f^3    t_f^4    t_f^5]; % 2.5
d_dot_f =     [0  1   2*t_f   3*t_f^2  4*t_f^3  5*t_f^4]; % 0
d_ddot_f =    [0  0   2     6*t_f    12*t_f^2  20*t_f^3]; % 0

A = [d_i; d_dot_i; d_ddot_i; d_f; d_dot_f; d_ddot_f];

B = [-2.5; 0; 0; 2.5; 0; 0];

X = linsolve(A,B);

a0 = X(1);
a1 = X(2);
a2 = X(3);
a3 = X(4);
a4 = X(5);
a5 = X(6);

t = t_i:0.1:t_f;

latPos =  1*a0 +  t*a1  + a2*t.^2 +  a3*t.^3  +  a4*t.^4  +  a5*t.^5;
latSpeed =  a1  + 3*a2*t +  3*a3*t.^2  +  4*a4*t.^3  +  5*a5*t.^4;
latAcc =   2*a2 +  6*t*a3  +  12*a4*t.^2  +  20*a5*t.^3;

plot(latAcc)











