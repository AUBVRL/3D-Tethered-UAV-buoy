
Ts = 1/500;              % sec, Simulation step time.
g = 9.81;               % gravity constant (m/s2)

%% Initalize:
alpha_bar = pi/4;       % cable angle
V_bar = 4;              % desired mean velocity
eps_V = 0.1;            % velocity margine to switch controller state (m/s).
T_init = 10;            % time before velocity control starts (s)
%% Wave characteristics
% AWTEC 2016: beirut, H = 2*A = 0.3 m, T = 3.5 s
% Wiki: fully developped, H =0.27 m, T = 3.0 s
%       fully developped, H =0.61 m, T = 4.0 s
%       fully developped, H =1.2  m, T = 5.0 s
% Wiki: fully developped, H =1.5  m, T = 5.7 s
%       fully developped, H =3.3  m, T = 8.0 s
% Wiki: fully developped, H =4.1  m, T = 8.6 s
% Wiki: Interpolate: H =2 m, T = 6.3 s
nu_w = 1.787*10^-6;   % kinematic viscosity of water (m2/s)
H = 10;               % mean water level (m)

% All waves in vector form:
wave.A = [1*3.3 ; 1*1.2]/2;          % wave height (m)
wave.T = [8 ; 5];            % wave period (s)
wave.omega = 2*pi./wave.T;   % wave circular frequency (rad/s)
wave.k = wave.omega.^2./g;    % wave number (rad/m)
wave.epsilon = [0 ; 0]*2*pi;        % random phase angle (rad)
wave.speed = wave.omega./wave.k;    % wave speed (m/s)
wave.lambda = wave.speed.*wave.T;   % wave length (m)
wave.psi = [0; 0];           % wave direction ([0 2pi] rad)
% wave 1 (to be removed)
wave1.A = 1*3.3/2;          % wave height (m)
wave1.T = 8;            % wave period (s)
wave1.omega = 2*pi/wave1.T;   % wave circular frequency (rad/s)
wave1.k = wave1.omega^2/g;    % wave number (rad/m)
wave1.epsilon = pi/2;        % random phase angle (rad)
wave1.speed = wave1.omega/wave1.k;    % wave speed (m/s)
wave1.lambda = wave1.speed*wave1.T;   % wave length (m)
wave1.psi = 0;           % wave direction ([0 2pi] rad)
% wave 2 (to be removed)
wave2.A = 1*1.2/2;          % wave height (m)
wave2.T = 5;            % wave period (s)
wave2.omega = 2*pi/wave2.T;   % wave circular frequency (rad/s)
wave2.k = wave2.omega^2/g;    % wave number (rad/m)
wave2.epsilon = pi;        % random phase angle (rad)
wave2.speed = wave2.omega/wave2.k;    % wave speed (m/s)
wave2.lambda = wave2.speed*wave2.T;   % wave length (m)
wave2.psi = 0;           % wave direction ([0 2pi] rad)
%
%
% A_w2 = 1*2.4/2;        % wave height (m)
% w_dir2 = -1;           % wave direction (1 for 0 deg, -1 for 180 deg)
% T_w2 = 7;            % wave period (s)
% omega_w2 = 2*pi/T_w2;   % wave circular frequency (rad/s)
% k_w2 = omega_w2^2/g;    % wave number (rad/m)
% epsilon_w2 = pi;        % random phase angle (rad)
%
% A_w2 = 1*3.3/2;        % wave height (m)
% w_dir2 = -1;           % wave direction (1 for 0 deg, -1 for 180 deg)
% T_w2 = 8;            % wave period (s)
% omega_w2 = 2*pi/T_w2;   % wave circular frequency (rad/s)
% k_w2 = omega_w2^2/g;    % wave number (rad/m)
% epsilon_w2 = pi;        % random phase angle (rad)


%% Water Current:
beta = 0;
current.tidal = -1*0.5*1;      % tidal current component (m/s)
current.wind  = 0*0.2;         % local wind current component (m/s)
current.beta = 0;
current.total = current.tidal + current.wind;
current.u = current.total*cos(current.beta);
current.v = current.total*sin(current.beta);

%% floating buoy: two cylinder shape buoys rigidly connected
% the 2 cylinder structure maitain the stability of the buoy
% and prevent roll motion
rho_w = 1000;   % water density (kg/m3)

% Cylinder:
% R_b = 0.1;             % buoy radius (m), cylinder
% A_b = pi*R_b^2;        % (m^2)
% l_b = 0.6;             % buoy length (m)
% m_b = 5;               % (kg)
% rho_b = m_b/(A_b*l_b); %  buoy density (kg/m3)
% J_b = m_b*R_b^2;

% A_im = (m_b/2)/rho_w/l_b; % this is only for 1 one of 2 cylinders
% theta = fzero(@(theta) R_b^2/2*(theta-sin(theta))-A_im,0);
% zeta_im = R_b*(1-cos(theta/2)); % R-H, water surface to buoy tip
% 
% u_s = A_w^2*omega_w*k_w*exp(2*k_w*(-zeta_im/2)); % stocks drift (m/s)
% 
% X0_b = [0,H+R_b-zeta_im];
% V0_b = [omega_w*A_w*cos(k_w*X0_b(1)+epsilon_w), 0];

% Cuboid:
buoy.h = 0.25;            % buoy side (m), cuboid 
buoy.A = buoy.h^2;           % (m^2)
buoy.l = 0.8;             % buoy length (m)
buoy.Vol = buoy.A*buoy.l;       % buoy volume (m)
% for the buoy to be half immersed:
buoy.m = rho_w*buoy.Vol/4;   % buoy mass (kg)
buoy.J = [buoy.m*buoy.h^2/12, buoy.m*buoy.h*buoy.l/12, buoy.m*buoy.h*buoy.l/12 ]; % buoy moment of inertia [kg.m2]

buoy.A_im = (buoy.m)/rho_w/buoy.l; % immersed frontal area (m^2) (different from whetted area)
buoy.Delta_h = (buoy.A_im/buoy.A-0.5)*buoy.h; % z_b-zeta, buoy center above water surface 

u_s1_x = cos(wave1.psi)*wave1.A^2*wave1.omega*wave1.k*exp(2*wave1.k*(-buoy.Delta_h/2)); % stocks drift (m/s)
u_s1_y = sin(wave1.psi)*wave1.A^2*wave1.omega*wave1.k*exp(2*wave1.k*(-buoy.Delta_h/2)); % stocks drift (m/s)

u_s2_x = cos(wave2.psi)*wave2.A^2*wave2.omega*wave2.k*exp(2*wave2.k*(-buoy.Delta_h/2)); % stocks drift (m/s)
u_s2_y = sin(wave2.psi)*wave2.A^2*wave2.omega*wave2.k*exp(2*wave2.k*(-buoy.Delta_h/2)); % stocks drift (m/s)

X0_b = [0,0,H-buoy.Delta_h];
% buoy initial velocity simillar as local wave velocity
u_w1 = cos(wave1.psi)*wave1.omega*wave1.A*sin(-wave1.k*X0_b(1)*cos(wave1.psi)-wave1.k*X0_b(2)*sin(wave1.psi)+wave1.epsilon);
v_w1 = sin(wave1.psi)*wave1.omega*wave1.A*sin(-wave1.k*X0_b(1)*cos(wave1.psi)-wave1.k*X0_b(2)*sin(wave1.psi)+wave1.epsilon);
w_w1 =             wave1.omega*wave1.A*cos(-wave1.k*X0_b(1)*cos(wave1.psi)-wave1.k*X0_b(2)*sin(wave1.psi)+wave1.epsilon); % ???stocks drift (m/s)
u_w2 = cos(wave1.psi)*wave1.omega*wave1.A*sin(-wave1.k*X0_b(1)*cos(wave1.psi)-wave1.k*X0_b(2)*sin(wave1.psi)+wave1.epsilon);
v_w2 = sin(wave1.psi)*wave1.omega*wave1.A*sin(-wave1.k*X0_b(1)*cos(wave1.psi)-wave1.k*X0_b(2)*sin(wave1.psi)+wave1.epsilon);
w_w2 =                wave1.omega*wave1.A*cos(-wave1.k*X0_b(1)*cos(wave1.psi)-wave1.k*X0_b(2)*sin(wave1.psi)+wave1.epsilon); % ???stocks drift (m/s)
u_w = u_w1 + u_w2;
v_w = v_w1 + v_w2;
w_w = w_w1 + w_w2;
V0_b = [current.u+u_w,current.v+v_w, w_w].*[1 1 1]; % check for correctness (body frame or inertial frame?)

%Cd_f =3; % drag
%drag_wave = 0.3; % wave lateral drag force (N)

buoy.a_11 = 0.05*buoy.m;        % added mass
buoy.a_22 = buoy.a_11;
buoy.a_33 = buoy.m;
buoy.b_11 = 0;               % added dampind
buoy.b_22 = 0;               % added dampind
omega_h = wave1.omega;
buoy.b_33 = 2*buoy.m*omega_h;   

C_S = [5;5;9]*10^-3;       % skin friction constant
D_s = [4;4;0.3*3];         % sfkin friction coefficient (viscous)
D_x = buoy.b_11 + D_s(1);     % total drag coefficient
D_x = buoy.b_22 + D_s(2);    
D_z = buoy.b_33 + D_s(3);

%% Cable:
Lc_0 = 7;  % 7-12              % Cable free length (m)
dL_max = 0.0;            % maximum cable elongation (m)
T_max = 80;              % max cable tension (N)
Kc = T_max/dL_max;       % Cable spring constant (N/m)

alpha_bar_0 = 45*(pi/180);
%zq_bar = X0_b(2) + Lc_0*tan(alpha_bar_0)*sqrt(1-cos(alpha_bar_0)^2)
zu_bar = X0_b(2) + Lc_0*sin(alpha_bar_0);
r_bar  = Lc_0-0.3;
xu_rel_bar  = 0.8*Lc_0*cos(alpha_bar_0);
r_min = (zu_bar-H)-0.2; % minimum allowed radial position (m)

X0_u = X0_b + [3/7 0 3/7]*Lc_0; % make alpha_ref_0 = 45.297 deg
V0_u = V0_b;
r_0 = sqrt( (X0_b(1) - X0_u(1))^2 +(X0_b(2) - X0_u(2))^2 + (X0_b(2) - X0_u(2))^2 );

%% Tension and azimuth control:
kp_a = 9.6; % 10 12
kd_a = 5.6; % 10 7
ki_a = 1.6;
kp_T = 25;
kd_T = 0.0;
ki_T = 12;

kp_r = 45;
kd_r = 19.5; % 7
ki_r = 9;  % 3

% PID controller:
kp_z = 3;
kd_z = 2;
ki_z = 1;
% Outer Loop (Forward/Backward) Controller
kp_x = 7;
kd_x = 5; % 5
ki_x = 1.2; 

epsilon_e_u1 = 0.05; %minimum control error to switch the controller (N). 
%% quadrotor UAV:

uav.J = diag([0.03,0.03,0.04]);              % moment of inertia (kg.m2)
uav.m = 1.8  ;             % quadcopter mass (kg)  1.634
uav.K = 40; % 20                  % motors maximum thrust,each (N) % 20 40
uav.n_motors = 4;            % number of motors
uav.L = 0.2;                 % qadrotor arm length (m)
if uav.n_motors == 4
    uav.torque_corr = 2*cosd(45);
elseif uav.n_motors == 6
    uav.torque_corr = 2*cosd(60)+1;
end

uav.euler0 = [0,0,0]; %10*pi/180;
Theta_mean = 0*atan2(V_bar*D_x*cos(alpha_bar_0),V_bar*D_x*sin(alpha_bar_0)+uav.m*g*cos(alpha_bar_0));%10*pi/180;

Tm = 1/20; % motor and rotor time constant (s)

%X0_q = X0_b+ [0,3];    % initial CG position (m)
%V0_q = [0,0];          % initial CG velocity (m/s) 

% Limits
%Xu_min= [-30 H];        % Window dimensions: min borders
%Xu_max= [30 20];        % Window dimensions: max borders
LIMIT_CMD_PITCH = 45*pi/180;
LIMIT_uCMD_HEIGHT = 0.9*uav.n_motors*uav.K; % 3.5*K
LIMIT_CMD_x = 3;

% Aerodynamics:
V_w = [-3 0 0];                 % wind velocity [m/2]
rho_a = 1.225;             % air density (kg/m^3)
% drag coefficient: square:1.05, sphere: 0.47, airfoil: 0.04
Cd_u = 0.2;               
% Ac = [0.0331 0.0331 0.05]; % Cross section area of the quadrotor in x and z direction (m^2)
Ac = 0.0331;
F_d_max_u = Cd_u*(0.5*rho_a*V_bar^2)*Ac
% Gains:

%
uav.kp_1 = 20; %k1 = 0.3  100
uav.kd_1 = 10;%k2 = 30   35
uav.ki_1 = 0.4;

% X:  
%k1 = 3.3; 
%k2 = 0.58; 
uav.kp_2 = 20; %k1 = 0.3  100
uav.kd_2 = 10;%k2 = 30   35
uav.ki_2=0.4;
% dc_M_x=0.3;
% Pitch: 
k3 = 24;%30; 4.5
k4 = 0.8;% 0.3;   2 
gamma_d_p=(2*0.3/0.03*2*0.2*12/4)/10; %k1=14.6
dc_M_p=6.7;

%
uav.kp_3 = 20/2; %k1 = 0.3  100
uav.kd_3 = 10/2;%k2 = 30   35
uav.ki_3=0.4/2;
%motors discrete transfer function:
sys=tf(1,[Tm 1]);
sysd = c2d(sys,Ts);
Am = sysd.denominator{1};
Bm = sysd.numerator{1};

%% Filters:

% smooth saturation functions:
Amp_sf = 3; % amplitude of the smoothing range
min_sf = 5; % lower bound of the smoothing function

% smooth saturation functions (for u_T radial position:
Amp_sf_rp = 0.05*uav.m*g; % amplitude of the smoothing range
min_sf_rp = 0.35*uav.m*g; % lower bound of the smoothing function

% Low pass filter:

A_1df = [1/40 1];                   % filter for 1 derivative
A_2df = conv([1/40 1],[1/60 1]);    % filter for 2 derivatives

sys=tf(1,A_1df);
sysd = c2d(sys,Ts);
Am1_LPF = sysd.denominator{1};
Bm1_LPF = sysd.numerator{1};

sys=tf(1,A_2df);
sysd = c2d(sys,Ts);
Am2_LPF = sysd.denominator{1};
Bm2_LPF = sysd.numerator{1};

% Z filter
sys=tf(1,conv([1/20 1],[1/20 1]));
sysd = c2d(sys,Ts);
AmZ_LPF = sysd.denominator{1};
BmZ_LPF = sysd.numerator{1};

% V_bar filter

sys=tf(1,conv([3 1],conv([3 1],[3 1]))); % 2 5 3
sysd = c2d(sys,Ts);
AmV_LPF = sysd.denominator{1};
BmV_LPF = sysd.numerator{1};

% alpha_dot filter
sys=tf(1,conv([1/5 1],[1/5 1]));
sysd = c2d(sys,Ts);
AmAlpha_LPF = sysd.denominator{1};
BmAlpha_LPF = sysd.numerator{1};

% r_ref filter
sys=tf(1,conv(conv([0.5 1],[0.5 1]),conv([0.5 1],[0.5 1])));
sysd = c2d(sys,Ts);
Am_rref_LPF = sysd.denominator{1};
Bm_rref_LPF = sysd.numerator{1};
% r_ref 20 filter
sys=tf(1,conv(conv([0.3 1],[0.3 1]),conv([0.3 1],[0.3 1])));
sysd = c2d(sys,Ts);
Am_rref20_LPF = sysd.denominator{1};
Bm_rref20_LPF = sysd.numerator{1};

% r during L hit filter
sys=tf(1,conv(conv([0.05 1],[0.05 1]),conv([0.05 1],[0.05 1])));
sysd = c2d(sys,Ts);
Am_reL_LPF = sysd.denominator{1};
Bm_reL_LPF = sysd.numerator{1};

% rdot during L hit filter
sys=tf(1,conv(conv([0.005 1],[0.005 1]),conv([0.005 1],[0.005 1])));
sysd = c2d(sys,Ts);
Am_rdeL_LPF = sysd.denominator{1};
Bm_rdeL_LPF = sysd.numerator{1};

% alpha switch filter
sys=tf(1,conv(conv([1.2 1],[1.2 1]),conv([1.2 1],[1.2 1])));
sysd = c2d(sys,Ts);
Am_asw_LPF = sysd.denominator{1};
Bm_asw_LPF = sysd.numerator{1};

% alpha switch smoothing filter
% sys=tf(1,conv([1 0],conv(conv([0.5 1],[0.5 1]),conv([0.5 1],[0.5 1]))));
sys=tf(1,conv(conv([0.15 1],[0.15 1]),conv([0.15 1],[0.15 1])));
sysd = c2d(sys,Ts);
Am_assw_LPF = sysd.denominator{1};
Bm_assw_LPF = sysd.numerator{1};

% coupled filter
sys=tf(1,conv([0.1 1],[0.1 1]));
sysd = c2d(sys,Ts);
Am_coup_LPF = sysd.denominator{1};
Bm_coup_LPF = sysd.numerator{1};

% fast coupled filter
sys=tf(1,conv([0.01 1],[0.01 1]));
sysd = c2d(sys,Ts);
Am_fast_coup_LPF = sysd.denominator{1};
Bm_fast_coup_LPF = sysd.numerator{1};
%% estimator filters:
estimation();

%%
V_bar0 = 20.8;
epsilon_1 = 5;        % cable tension lower bound (N)
epsilon_2 = 0.05*buoy.Vol;
A_whetted =  buoy.l*buoy.h + 2*buoy.l*(buoy.h/4) % average
D_from_C = A_whetted*V_bar0*0.5*rho_w*C_S(1)
v_x_w_max = wave1.omega*wave1.A;
T_bar0 = D_from_C*V_bar0/cos(alpha_bar)


V_min = epsilon_1*cosd(alpha_bar)/D_from_C + current.total + v_x_w_max
V_max = (buoy.m + uav.m - epsilon_2*rho_w)*g*tan(uav.euler0(2))...
            /D_from_C+current.total+u_w+u_s1_x+abs(v_x_w)

%% Frequency analysys:
A_c = l_b*h_b;
D_z0 = 55;
V_bar01 = 13;
V_bar02 = -5;

omega_b = sqrt((rho_w*g*A_c/(m_b+a_33)));
zeta_b = D_z0/(2*sqrt((m_b+a_33)*rho_w*g*A_c));
omega_e11 = omega_w - omega_w^2*V_bar01/g*w_dir; % -0.508 1.72
omega_e12 = omega_w2 - omega_w2^2*V_bar01/g*w_dir; % -0.508 1.72

omega_e21 = omega_w - omega_w^2*V_bar02/g*w_dir; % -0.508 1.72
omega_e22 = omega_w2 - omega_w2^2*V_bar02/g*w_dir; % -0.508 1.72

Gb_s = tf(omega_b^2,[1 2*zeta_b*omega_b omega_b^2])
% figure(1)
% bode(Gb_s)
% grid on
i=0;
for w = 0:0.1:20
    i = i+1;
    [mag0,phase,wout] = bode(Gb_s,w); % mag0 is not in dB!
    mag(i,1) = (mag0); % db2mag(12.1)
     % 20*log10(2)
    OMEGA = w/omega_b;
    mag2(i,1) = (509.7/rho_w*g*A_c)/sqrt((1 - OMEGA^2)^2+(2*zeta_b*OMEGA)^2);
end

% we1_C3 = 0.5; we2_C3 = 3.7; 
% we1_C4 = 1.7; we2_C4 = 4.3; 
% OMEGAe1_C3 = 0.056; OMEGAe2_C3 = 0.41;
% OMEGAe1_C4 = 0.19;  OMEGAe2_C4 = 0.48;
% A1 = 0.75; A2 = 0.135;
% dh1_C3 = 0.23 ; dh2_C3 = 2.6; % cm
% dh1_C3 = 2.7 ; dh2_C3 = 3.8; % cm

% OMEGA = 0.19;
% A = 0.75;
% Mag = 100*A*((509.7/rho_w*g*A_c)/sqrt((1 - OMEGA^2)^2+(2*zeta_b*OMEGA)^2)-1) % cm
% 
% w = [0:0.1:20];
% figure(2)
% hold on
% plot(w./omega_b,mag)
% plot(w./omega_b,mag2)
% grid on
% omega_e/omega_b