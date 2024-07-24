clc;clear all;close all;fclose all;format compact;
syms d1 d2 d3 d4 x y th W H r wr wl b dt
%{
d1, d2, d3 and d4 -> Sensor distance readings
x, y, th -> x, y position and angle theta
W and H -> Width and height of the square arena
r -> Wheel radius
wr and wl -> Angualr velocities of the right and left wheels
b -> Baseline distance between left and right wheels
%}

X = [x; y; th];
vr = r*wr;
vl = r*wl;
w = (vr - vl) / b;              % Angular velocity of robot around instantanious centre of rotation (ICR)
R = b/2 * (vr+vl)/(vr-vl);      % Radius of ICR
V = w*R;                        % Instantanious velocity of robot

f = X + dt*[V*cos(th);V*sin(th);w];
A = jacobian(f,X);

% Solutions for looking at each wall
Dt = (H-y)/cos(th);         % Dist to top
Dr = (W-x)/cos(th);         % Dist to right
Db = y/cos(th);             % Dist to back
Dl = x/cos(th);             % Dist to left

[a1, a2, a3, a4] = angles_to_corners(x,y,W,H);

if(th > a2 && th < a1)
    % Looking at right wall
elseif(th > a1 && th < a4)
    % Looking at top wall
elseif(th > a4 && th < 3)
    % Looking at left wall
else
    % Looking at bottom wall
end
