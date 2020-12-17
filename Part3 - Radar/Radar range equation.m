%Operating frequency (Hz)
fc = 77.0e9;

%Transmitted power (W)
Pt = 3e-3;

%Antenna Gain (linear)
G =  10000;

%Minimum Detectable Power
Ps = 1e-10;

%RCS of a car
RCS = 100;

%Speed of light
c = 3*10^8;

lamda = c / fc;
disp(lamda);

range_max = nthroot( (Pt*G^2*lamda^2*RCS) / (Ps*(4*pi)^3), 4);
disp(range_max);
