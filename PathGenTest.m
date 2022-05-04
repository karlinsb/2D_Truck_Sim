%% Pream
% test script and function to generate 2D road pathways and the
% acceleration of a semi-tractor system with a parameterized dynamic model

clear
clf
%% Define Params
m=17000;
m_eff=m*1.05;
r_eff = .527;
T = 1500;
N = 2.64;

% load coefs
C_dl=1.225/2*8*0.8;
C_rr=0.006;
C_a =18;%kW


%% Traverse Path and solve for a,v,p?
[x,y,~] = pathGen(5/3,100,26400,0.25);

dx = diff(x);
dy = diff(y);
figure(1)
stackedplot([y(1:end-1)' dy'])

x_d = zeros(1,length(x));
x_d(1) = 20;
x_dd = zeros(1,length(x));
for i = 1:length(x)-1
    % Compute Inputs
    th = atan(dy(i)/dx(i));

    % drive force at tire
    F_e = T*N/r_eff;

    % rolling resistance 
    F_rr = C_rr*m*9.81*cos(th);
    
    % Aero Drag
    F_d = C_dl*x_d(i).^2;
    
    % Accessory Resistance
    % rpm = v*r_eff/N;
    % P_a = 2.91/100+1000
    F_a = C_a*10^3/x_d(i);
    
    % Grade Resistance
    F_g = m*9.81*sin(th);
    
    % x_dd = 1/m_eff*(T*N*r_eff - C_dl*x_d^2 - C_rr*m*9.81*cos(th))
    
    x_dd(i+1) = (F_e-F_d-F_rr-F_a-F_g)/m_eff;
    x_d(i+1) = x_d(5/3,100,26400);


end
%% new path function testing

sp = pathGen(5/3,100,26400);
x = 0:1:26400;
y = ppval(sp,x);
dx = diff(x);
dy = diff(y);
figure(1)
stackedplot([y(1:end-1)' dy'])

%% Path Generation Function
function sp = pathGen(h_std, spacing, dist)
% pathGen(), 1D Gaussian Random-Walk Path Generator
% Generates a 1D random-walk of length dist with constant spacing and 
% std dev h_std. outputs a cubic interpolated spline as a piecewise
% polynomial structure
% 
% pathGen(5,100,1500) will output an interpolated piecewise polynomial over
% 1500 units with an input spacing of 100 units, an output spacing of
% 0.5 units, and a standard deviation between input points of 5 units.
%
% See also: INTERP1 PPVAL SPLINE

numSam = dist/spacing;
    x = linspace(0,dist,numSam);
%     y = abs(h_std*randn(1,numSam));
    y = zeros(1,numSam);
    dy = zeros(1,numSam);
    for i = 1:numSam-1
        dy(i) = h_std*randn(1);
        y(i+1) = y(i)+dy(i);
    end

%     x_i = 0:interp:dist;
    sp = spline(x,y);

%     plot(x,y,'o',x_i,sp)
end
