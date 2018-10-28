clc
clear all

vw= 5;                %cross wind speed (m per sec)
vs= 11.4;             %stall velocity (m per sec)
u1= 1.1*vs;           %approach velocity 
vt= sqrt(u1^2 + vw^2);%total speed
sf= 0.7*0.15;         %fuselage side area // enter side area in syntax [length*height]
sv= 0.131;             %vertical tail side area
ss=1.02*(sf+sv);      %total side area
lf=0.7 ;              %length of fuseage
df=0.15;              %width of fuselage
xf=0.35;              %distance of the centre of the fuselage from the nose
xv= 1.99375 ;                %distance of the centre of the vertical tail measured from the nose
xca=(((lf*df)*xf)+(sv*xv))/((lf*df)+sv);
xcg= 0.4 ;               %cg distance measured from the nose(m)
dc=xca-xcg;
lv=1.39075;             %most forward cg to ac of Vstab distance(m)
p=1.225;              %take p(density of air)=1.225
cdy=0.6;              %take cdy=0.6
fw=0.5*p*vw*vw*ss*cdy %side force of the aircraft
B= 1;                 %br/bv(span rudder/span Vstab)
C=0.4;                %cr/cv(chord rudder/chord Vstab)
beta= atan(vw/u1); %side-slip angle (in radians)
kf1= 0.75;  %range 0.65 to  0.85
Cl_alpha_v=6.11     %from xflr
ssg= 0;     %side slip gradient
Nv= 0.96;      %dynamic pressure ratio (range 0.95 to 1)
lvt=1.24 ;      %distance btw the AC of wing to Ac of V-stab
b= 2.8;        %wing span
s=1.3 ;        %wing area
kf2= 1.35;      %range 1.3 to 1.4
tr= 0.6;        %from graph using cr/cv
vv= 0.045;       %vertical tail volume coeff (range 0.03 to 0.06)

Cnb= (kf1*Cl_alpha_v*(1- ssg)*Nv*lvt*sv)/(b*s)
Cyb= ((-kf2)*Cl_alpha_v*(1-ssg)*Nv*sv)/s
Cndr= (-Cl_alpha_v)*vv*Nv*tr*B
Cydr= (Cl_alpha_v*Nv*tr*B*sv)/s

syms x y;
e1= ( ((vw^2)*ss*cdy)/((vt^2)*s) ) - ( Cyb*(beta - x) + Cydr*y );
x= solve(e1,x);      %writing eq1 in in terms of 'y'
e2= 0.5*p*(vt^2)*s*b*(Cnb*(beta - x) + Cndr*y) + fw*dc*cos(x);
y= solve(e2,y);      %obtaining the value of 'y'
syms x;           %erasing the value of x written in terms of y
e2= 0.5*p*(vt^2)*s*b*(Cnb*(beta - x) + Cndr*y) + fw*dc*cos(x);
x= solve(e2,x);      %obtaining the value of x
[x y]
%fprintf('x= %.4f and y= %.4f \n', x, y);


