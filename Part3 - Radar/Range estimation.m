
%Speed of light
c = 3*10^8;

%The radar maximum range = 300m
Rmax = 300;

Dres = 1;

%Sweep frequency
Bs = c / (2*Dres);

%The sweep time
Ts = (5.5*2*Rmax) / c;

fb = [0 1.1e6 13e6 24e6];

for i=1:4
  range = (c*Ts*fb(i)) / (2*Bs); 
  disp(range);
endfor