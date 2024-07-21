% Robot 3R
l1 = 2;
l2 = 2;
l3 = 1;
L(1) = Link('revolute','alpha',0,'a',l1,'d',0,'qlim',[-3*pi/4 3*pi/4 ],'offset',0);
L(2) = Link('revolute','alpha',0,'a',l2,'d',0,'qlim',[-3*pi/4  3*pi/4 ],'offset',0);
L(3) = Link('revolute','alpha',0,'a',l3,'d',0,'qlim',[-3*pi/4  3*pi/4 ],'offset',0);
RRR = SerialLink(L,'name','3R');
RRR.plot([0 0 0],'top')

x = 3;
y = 3;
phi = pi/2;

x0 = x - l3*cos(phi);
y0 = y - l3*sin(phi);
codo = 1;
if codo
    [qinv, check] = inv_kin_2R(x0,y0,l1,l2,1);
    q3 = phi - qinv(1) - qinv(2);
    q = [qinv; q3]';
else
    [qinv, check] = inv_kin_2R(x0,y0,l1,l2,0);
    q3 = phi - qinv(1) - qinv(2);
    q = [qinv; q3]';
end

T = RRR.fkine(q);
RRR.plot(q,'top')
text(x+0.2,y,strcat('[',num2str(T(1,4)),',',num2str(T(2,4)),']'))


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calcula la cinematica inversa para un mecanismo 2 R
% Entradas:
% x = Vector cartesiano de X
% y = Vector cartesiano de Y
% L1 =
% L2 =
% codo = Codo arriba (1), codo abajo (~1)
% Salidas:
% q = Valores angulares para las dos articulaciones
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [q, check] = inv_kin_2R(x,y,L1,L2,codo)

num = x.^2+y.^2-L1^2-L2^2;
den = 2*L1*L2;
D = num./den;
check = (D<=1);

if check
    q2 = atan2(-sqrt(1-D.^2),D);
    
    art = codo;
    if art == 1
        q2 = atan2(sqrt(1-D.^2),D);
    end
    
    q1 = atan2(y,x) - atan2(L2*sin(q2), L1+L2*cos(q2));
    
    q = [q1; q2];
else
    %warning('No real solution')
    q = [NaN NaN];
end
end
