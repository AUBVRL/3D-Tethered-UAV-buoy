%% Filters
% T=1/500; %sec, Simulation step time.
Tg = 1/ 20;  % GPS sampling time.
Tz = 1/500; % Noise sampling time
Ts = 1/500; % IMU Sample time
Tn = 1/500; % Notch filters sampling time
Tc = 1/500; % Controller sampling time

Tsc = 1/100; % sterio camera sampling time


% Second order filter parameters for generating rate data
OMEGA_DIFF = 60;            % (rad/s) - For generating rate data
OMEGA_FILTER = 60;          % (rad/s) - For smoothing data
OMEGA_FILTER_OUTER = 40;    % (rad/s) - For smoothing optitrack x and z data

X_GPS_offset = [0.01 0.005];  % GPS offset error from true CG. [m]

% Notch:
% an1=[0.337540151883547,-0.310253311913592,0.337540151883547];
% bn1=[1,-0.310253311913592,-0.324919696232906];
% an2=[0.327054913984048,0.404148045226953,0.327054913984048];
% bn2=[1,0.404148045226953,-0.345890172031905];
% an3=[0.420807779837732,0.806706851924986,0.420807779837732];
% bn3=[1,0.806706851924986,-0.158384440324536];

%
an1=[0.337540151883547,-0.271994199011296,0.337540151883547];
bn1=[1,-0.271994199011296,-0.324919696232906];
an2=[0.327054913984048,0.404148045226953,0.327054913984048];
bn2=[1,0.404148045226953,-0.345890172031904];
an3=[0.420807779837732,0.806706851924986,0.420807779837732];
bn3=[1,0.806706851924986,-0.158384440324536];

an1_v=[0.4208 -0.8097 0.4208];
bn1_v=[1 -0.8097 -0.1584];


% Resample notchs for Ts ~= 1/500 s:

temp = d2d(tf(an1,bn1,1/500),Tn);
an1 = temp.numerator{1};
bn1 = temp.denominator{1};
%
temp = d2d(tf(an2,bn2,1/500),Tn);
an2 = temp.numerator{1};
bn2 = temp.denominator{1};
%
temp = d2d(tf(an3,bn3,1/500),Tn);
an3 = temp.numerator{1};
bn3 = temp.denominator{1};
%
temp = d2d(tf(an1_v,bn1_v,1/500),Tn);
an1_v = temp.numerator{1};
bn1_v = temp.denominator{1};

% c2d:

%motors discrete transfer function:
sys=tf(1,[Tm 1]);
sysd = c2d(sys,Ts);
Am = sysd.denominator{1};
Bm = sysd.numerator{1};

% High pass filters:
sys=tf([OMEGA_FILTER 0],[OMEGA_FILTER 1]);
sysd = c2d(sys,Ts);
Am_HPF = sysd.denominator{1};
Bm_HPF = sysd.numerator{1};

% Low pass filter:
sys=tf([OMEGA_FILTER],[1 OMEGA_FILTER]);
sysd = c2d(sys,Ts);
Am_LPF = sysd.denominator{1};
Bm_LPF = sysd.numerator{1};

OMEGA_FILTER2 = 15;    % (rad/s) - For smoothing optitrack x and z data
% Low pass filter:
sys=tf([OMEGA_FILTER2],[1 OMEGA_FILTER2]);
sysd = c2d(sys,Ts);
Am_LPF2 = sysd.denominator{1};
Bm_LPF2 = sysd.numerator{1};