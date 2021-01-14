%% Industrial Robotics
clear; close all; clc;

%% Direct Kinematics
syms a f d t a3 a4 d3 t1 t2 t3 t4 t5 t6 real

A = [cos(t) -sin(t)*cos(f) sin(t)*sin(f) a*cos(t);
     sin(t) cos(t)*cos(f) -cos(t)*sin(f) a*sin(t);
       0       sin(f)         cos(f)        d;
       0         0              0           1];

a3=44;
a4=40;
d3=44; 

DH = [0  pi/2 0  t1;
      0 -pi/2 0  t2;
      a3   0  d3 t3;
      a4   0  0  t4;
      a3/2  pi/2 0  t5;
      a3/2    0  0  t6];
  
n=size(DH,1); %Number of joints


Rob = SerialLink(DH,'name','Rob');
Rob.plot([ pi/2 pi/2 0 0 0 pi/2]);
Rob.teach

% Intermediate transformation Matrices
for i=1:n
    eval("A"+int2str(i)+int2str(i-1)+"=subs(A,[a f d t],DH(i,:));");
end
% Multiplication of all the A_i^i-1 and extraction of the z_i-1 e p_i-1
z0 = [0;0;1];
p0 = [0;0;0];
z1 = A10(1:3,1:3)*z0;
p1 = A10(1:3,2:4)*z0;
eval("T"+int2str(n)+"0=A10;");
for i=2:n
    eval("T"+int2str(n)+"0=T"+int2str(n)+"0*A"+int2str(i)+int2str(i-1)+';');
    eval("z"+int2str(i)+"=T"+int2str(n)+"0(1:3,1:3)*z0;");
    eval("p"+int2str(i)+"=T"+int2str(n)+"0(1:3,2:4)*z0;");
end

eval("T"+int2str(n)+"0s=simplify(T"+int2str(n)+"0)");

%% Inverse Differential Kinematics

% Geometric jacobian
J = [];
% under assumption of rotary joints only
for i=1:n
    eval("J = [J , [cross(z"+int2str(i-1)+",(p"+int2str(n)+"-p"+int2str(i-1)+")) ; z"+int2str(i-1)+"]];");
end
J = simplify(J);

simplify(det(J))

%% Robot simulation
% Parameters setting
a3=44;
a4=40;
d3=44;

t1=zeros(1,101);
t2=pi/2*ones(1,101);
t3=t1;
t5=[0:2*pi/100:2*pi];
t4=asin(a4/d3*cos(t5))-t5;
t6=t1;

Q=[t1;t2;t3;t4;t5;t6];

% Base frame
Tb0 = [1  0  0  0;
       0 -1  0  0;
       0  0 -1  0;
       0  0  0  1];
% Links generation (under assumption of rotary joints only)
for i=1:n
    eval("l"+int2str(i)+"=Link('a',"+int2str(eval(DH(i,1)))+",'alpha',"+int2str(eval(DH(i,2)))+",'d',"+int2str(eval(DH(i,3)))+");");
end
% Structure generation and plot
Rob = SerialLink([l1 l2 l3 l4 l5 l6],'name','Rob');
Rob.base = Tb0;
Rob.plot(Q','fps',30,'movie','hey3.mp4');
