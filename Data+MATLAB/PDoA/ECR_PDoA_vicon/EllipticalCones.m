clear all
close all
% https://math.stackexchange.com/questions/2206137/a-right-elliptical-cone-is-4m-high-and-has-an-elliptical-base-with-half-axes-len
% https://www.mathworks.com/matlabcentral/answers/349988-how-to-create-a-3d-cone
% https://mathworld.wolfram.com/EllipticCone.html
% A cone with elliptical cross section. The parametric equations for an elliptic cone of height h, semimajor axis a, and semiminor axis b are
% x	=	a( (h-u)/h )cos(v)	
% y	=	b( (h-u)/h )sin(v)	
% z	=	u,	
% where v in [0,2pi) and u in [0,h].

% tan(semimajor angle) = a/h =>  a = h*tan(semimajor angle)
% tan(semiminor angle) = b/h =>  b = h*tan(semiminor angle)

deg_SM = 30; %semimajor angle in degrees by experiment: measured extreme range is +/- 30deg @ 3m
deg_sm = 7; %semiminor angle in degrees by experiment: measured extreme range is +/-  7deg @ 3m
rad_SM = deg2rad(deg_SM); %semimajor angle in radians
rad_sm = deg2rad(deg_sm); %semiminor angle in radian
h = 3.1; %experimental range in meters
a = h*tan(rad_SM);
b = h*tan(rad_sm);
a1 = a;
b1 = b;
a2 = b;
b2 = a;

% % Test values to verify
% a = 7;
% b = 2.5;
% h = 9;

% u = linspace(0,h) ;
% v = linspace(0,2*pi) ;
% [U,V] = meshgrid(u,v) ;
% x = (a/h)*(h-U).*cos(V) ;
% y = (b/h)*(h-U).*sin(V) ;
% z = U - h; %subtract h to place vertex at origin
% surf(x,-z,y) %switch y and z to point cone along the y-axis
% axis equal

u = linspace(0,h) ;
v = linspace(0,2*pi) ;
[U1,V1] = meshgrid(u,v) ;
[U2,V2] = meshgrid(u,v) ;
x1 = (a1/h)*(h-U1).*cos(V1) ;
y1 = (b1/h)*(h-U1).*sin(V1) ;
x2 = (a2/h)*(h-U1).*cos(V1) ;
y2 = (b2/h)*(h-U1).*sin(V1) ;
z = U1 - h; %subtract h to place vertex at origin

% mesh(x1,-z,y1) %switch y and z to point cone along the y-axis
% hold on
% mesh(x2,-z,y2) %switch y and z to point cone along the y-axis
% axis equal

contour(x1,y1,z) %switch y and z to point cone along the y-axis
hold on
contour(x2,y2,z) %switch y and z to point cone along the y-axis
axis equal

