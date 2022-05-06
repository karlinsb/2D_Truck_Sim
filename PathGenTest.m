%% Pream
% test script and function to generate 2D road pathways and the
% acceleration of a semi-tractor system with a parameterized dynamic model

clear
% clf
load('rng.mat');
rng(s);

%% Define Params
m=17000;
m_eff=m*1.05;
r_eff = .527;
T_max = 1000;
N = 2.64;

% load coefs
C_dl=1.225/2*8*0.8;
C_rr=0.006;
C_a =18;%kW


%% Traverse Path and solve for a,v,p?
pathLen = 26400;
sp = pathGen(5/3,100,pathLen);
x = 0:1:pathLen;
y = ppval(sp,x);
dx = diff(x);
dy = diff(y);
% figure(1)
% stackedplot([y(1:end-1)' dy']);

dt = 0.1;
numSamp = 30*60/dt;
T = zeros(1,numSamp);
x = zeros(1,numSamp);
x_d = zeros(1,numSamp);
x_d(1) = 20;
x_dd = zeros(1,numSamp);

th = interp1(1:length(dy),dy,1:numSamp);
h(1) = 0;
for t = 1:numSamp-1
    v_des = 25;
    T(t) = 0.3*(v_des-x_d(t))*T_max; % dummy P copntroller
    T(t) = min(T_max, max(0,T(t))); % constrain [0, t_max]

    % Compute Forces
    F_e(t) = T(t)*N/r_eff;    % drive force at tire
    F_rr(t) = C_rr*m*9.81*cos(th(t)); % rolling resistance
    F_d(t) = C_dl*x_d(t).^2;   % Aero Drag    
    F_a(t) = C_a*10^3/x_d(t);  % Accessory Resistance
    % rpm = v*r_eff/N;
    % P_a = 2.91/100+1000
    F_g(t) = m*9.81*sin(th(t));   % Grade Resistance

    x_dd(t+1) = (F_e(t)-F_d(t)-F_rr(t)-F_a(t)-F_g(t))/m_eff;
    x_d(t+1) = x_d(t)+x_dd(t+1)*dt;
    x(t+1) = x(t) + (x_d(t+1)+x_d(t))/2*dt;

    % Compute Inputs
    h(t+1) = ppval(sp,x(t+1));
    
%     th(t+1) = atan((h(t+1)-h(t))/dt);
%     dy2 = interp1(1:length(dy),dy,1:numSamp)
%     th(t+1) = dy2(t+1);
end
figure(1)
stackedplot([interp1(1:pathLen,y(1:end-1),1:t+1)' th' interp1(1:pathLen,dy,1:t+1)' ...
    T' x_dd' x_d' x'],'DisplayLabels', ...
    ["Path" "\theta" "dy" "T" "acc" "vel" "pos"])

figure(2)
stackedplot([y(1:end-1)' interp1(1:t,F_e,1:pathLen)' interp1(1:t,F_rr,1:pathLen)' ...
            interp1(1:t,F_d,1:pathLen)' interp1(1:t,F_a,1:pathLen)'...
            interp1(1:t,F_g,1:pathLen)'],'DisplayLabels', ...
            ["Path" "F_e" "F_rr" "F_d" "F_a" "F_g"])
% 
% datOut = table(linspace(0,numSamp*dt,numSamp), th',x_dd',F_e' 'VariableNames',{'Time''Theta','Vel'});
% 
% save('pathData.mat',datOut,m,m_eff,)
%% new path function testing
% 
% sp = pathGen(5/3,100,26400);
% x = 0:1:26400;
% y = ppval(sp,x);
% dx = diff(x);
% dy = diff(y);
% figure(1)
% stackedplot([y(1:end-1)' dy'])

%% Path Generation Function
function pp = pathGen(h_std, spacing, dist)
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
    pp = spline(x,y);

%     plot(x,y,'o',x_i,sp)
end
