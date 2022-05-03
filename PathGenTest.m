%% Pream
% test script and function to generate 2D road pathways and the
% acceleration of a semi-tractor system with a parameterized dynamic model

%% Define Params
% m
% m_eff
% r_eff
% T
% N
% 
% C_dl
% C_rr
% C_a


%% Traverse Path and solve for a,v,p?
figure(1)
subplot(2,1,1)
[~,~,dy] = pathGen(5/3,100,26400,0.25);
subplot(2,1,2)
plot(dy)

%% Path Generation Function
function [x_i,y,dy] = pathGen(h_std, spacing, dist, interp)
% pathGen(), 1D Gaussian Random-Walk Path Generator
% Generates a 1D random-walk of length dist with constant spacing and 
% std dev h_std. outputs a cubic interpolated y with spacing interp
% 
% pathGen(5,100,1500,.5) will output an interpolated spline vector over
% 1500 units with an input spacing of 100 units, an output spacing of
% 0.5 units, and a standard deviation between input points of 5 units.

numSam = dist/spacing;
    x = linspace(0,dist,numSam);
%     y = abs(h_std*randn(1,numSam));
    y = zeros(1,numSam);
    dy = zeros(1,numSam);
    for i = 1:numSam-1
        dy(i) = h_std*randn(1);
        y(i+1) = y(i)+dy(i);
    end

    x_i = 0:interp:dist;
    sp = spline(x,y,x_i);

    plot(x,y,'o',x_i,sp)
end
