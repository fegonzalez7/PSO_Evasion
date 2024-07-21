function sol = inv_irb_140_m2(varargin)
% Parametros
% escala = 1;
% l = escala*[70 352 360 380 65]; Pasar de mm a m
% qlims = deg2rad([-180 180; -90 110; -230 50; -200 200; -115 115; -400 400]);
% T = transl(-400,-110,250)*troty(-pi/2)*trotz(-pi/16);
% IRB140 = robot
switch nargin
   % T = MTH a analizar
   % robot = Objeto robot creado en el RTB de Peter Corke (en DHmod)
   % qlims = Limites articulares
   % l = Longitues de eslabon
   case 2
      T = cell2mat(varargin(1));
      robot = varargin(2);
      IRB140 = robot{1};
      qlims = IRB140.qlim;
      l = [IRB140.links(2).a IRB140.links(1).d IRB140.links(3).a IRB140.links(4).d IRB140.tool(3,4)];
   case 3
      T = cell2mat(varargin(1));
      %robot = varargin(2);
      %IRB140 = robot{1};
      qlims = cell2mat(varargin(2));
      l = cell2mat(varargin(3));
end

% Salidas
qinv = zeros(8,6);
cnfgdata = zeros(8,4);
sol = struct('q',[],'cnfg',[]);

% Solucion de la inversa
% Posicion Wrist
Posw =  T(1:3,4) - l(5)*T(1:3,3);
Tw = T;
Tw(1:3,4) = Posw;

% Solucion para q1
q1a = atan2(Tw(2,4), Tw(1,4)); % Solucion +
q1b = atan2(-Tw(2,4),-Tw(1,4)); % Solucion -
% if abs(q1b) >= pi
%    q1b = -pi + q1a;
% end
qinv(1:4,1) = q1a;
qinv(5:8,1) = q1b;

% Usando q1a
% Plano articulaciones 2-3
pxy = sqrt(Tw(1,4)^2 + Tw(2,4)^2) - l(1);
z = Tw(3,4) - l(2);
r = sqrt(pxy^2 + z^2);
% Mecanismo 2R
the3 = acos((r^2 - l(3)^2 -l(4)^2)/(2*l(3)*l(4)));
alp = atan2(z,pxy);
if isreal(the3) % Valida que sea una solucion posible (real)
   the2d = alp - atan2(l(4)*sin(the3),l(3)+l(4)*cos(the3));
   the2u = alp + atan2(l(4)*sin(the3),l(3)+l(4)*cos(the3));
   % Codo Abajo
   q2ad =  pi/2 - the2d;
   q3ad = -(pi/2 + the3);
   % Codo Arriba
   q2au =  pi/2 - the2u;
   q3au = -(pi/2 - the3);
else
   q2au = NaN;
   q3au = NaN;
   q2ad = NaN;
   q3ad = NaN;
end
qinv(1:2,2:3) = [q2au q3au; q2au q3au];
qinv(3:4,2:3) = [q2ad q3ad; q2ad q3ad];

% Usando q1b
% Plano articulaciones 2-3
pxy = sqrt(Tw(1,4)^2 + Tw(2,4)^2) + l(1);
z = Tw(3,4) - l(2);
r = sqrt(pxy^2 + z^2);
% Mecanismo 2R
the3 = acos((r^2 - l(3)^2 -l(4)^2)/(2*l(3)*l(4)));
alp = atan2(z,pxy);
if isreal(the3)
   the2d = alp - atan2(l(4)*sin(the3),l(3)+l(4)*cos(the3));
   the2u = alp + atan2(l(4)*sin(the3),l(3)+l(4)*cos(the3));
   % Codo Abajo
   q2bd = -(pi/2 - the2d);
   q3bd = -pi + (pi/2 + the3);
   % Codo Arriba
   q2bu = -(pi/2 - the2u);
   q3bu = -pi + (pi/2 - the3);
else
   q2bd = NaN;
   q3bd = NaN;
   q2bu = NaN;
   q3bu = NaN;
end
qinv(5:6,2:3) = [q2bd q3bd; q2bd q3bd];
qinv(7:8,2:3) = [q2bu q3bu; q2bu q3bu];

if ~any(any(isnan(qinv(1:2,:))))
% Calculo del wrist au
T03au = irb140MthSeq([1 2 3],[qinv(1,1) qinv(1,2) qinv(1,3)],l)*transl(0,l(4),0);
R46au = T03au(1:3,1:3)'*Tw(1:3,1:3);
q4aul = atan2(R46au(3,3),-R46au(1,3));
qinv(1,4) = q4aul;
q4aur = atan2(-R46au(3,3),R46au(1,3));
qinv(2,4) = q4aur;
% Wrist izquierdo
T04aul = irb140MthSeq([1 2 3 4],[qinv(1,1) qinv(1,2) qinv(1,3) qinv(1,4)],l);
R56aul = T04aul(1:3,1:3)'*Tw(1:3,1:3);
q5aul = atan2(-R56aul(1,3),R56aul(3,3));
qinv(1,5) = q5aul;
T05aul = irb140MthSeq([1 2 3 4 5],[qinv(1,1) qinv(1,2) qinv(1,3) qinv(1,4) qinv(1,5)],l);
R6aul = T05aul(1:3,1:3)'*Tw(1:3,1:3);
q6aul = atan2(R6aul(1,2),R6aul(3,2));
qinv(1,6) = q6aul;
% Wrist derecho
T04aur = irb140MthSeq([1 2 3 4],[qinv(1,1) qinv(1,2) qinv(1,3) qinv(2,4)],l);
R56aur = T04aur(1:3,1:3)'*Tw(1:3,1:3);
q5aur = atan2(-R56aur(1,3),R56aur(3,3));
qinv(2,5) = q5aur;
T05aur = irb140MthSeq([1 2 3 4 5],[qinv(1,1) qinv(1,2) qinv(1,3) qinv(2,4) qinv(2,5)],l);
R6aur = T05aur(1:3,1:3)'*Tw(1:3,1:3);
q6aur = atan2(R6aur(1,2),R6aur(3,2));
qinv(2,6) = q6aur;
end

if ~any(any(isnan(qinv(3:4,:))))
% Calculo del wrist ad
T03ad = irb140MthSeq([1 2 3],[qinv(3,1) qinv(3,2) qinv(3,3)],l)*transl(0,l(4),0);
R46ad = T03ad(1:3,1:3)'*Tw(1:3,1:3);
q4adl = atan2(R46ad(3,3),-R46ad(1,3));
qinv(3,4) = q4adl;
q4adr = atan2(-R46ad(3,3),R46ad(1,3));
qinv(4,4) = q4adr;
% Wrist izquierdo
T04adl = irb140MthSeq([1 2 3 4],[qinv(3,1) qinv(3,2) qinv(3,3) qinv(3,4)],l);
R56adl = T04adl(1:3,1:3)'*Tw(1:3,1:3);
q5adl = atan2(-R56adl(1,3),R56adl(3,3));
qinv(3,5) = q5adl;
T05adl = irb140MthSeq([1 2 3 4 5],[qinv(3,1) qinv(3,2) qinv(3,3) qinv(3,4) qinv(3,5)],l);
R6adl = T05adl(1:3,1:3)'*Tw(1:3,1:3);
q6adl = atan2(R6adl(1,2),R6adl(3,2));
qinv(3,6) = q6adl;
% Wrist derecho
T04adr = irb140MthSeq([1 2 3 4],[qinv(3,1) qinv(3,2) qinv(3,3) qinv(4,4)],l);
R56adr = T04adr(1:3,1:3)'*Tw(1:3,1:3);
q5adr = atan2(-R56adr(1,3),R56adr(3,3));
qinv(4,5) = q5adr;
T05adr = irb140MthSeq([1 2 3 4 5],[qinv(3,1) qinv(3,2) qinv(3,3) qinv(4,4) qinv(4,5)],l);
R6adr = T05adr(1:3,1:3)'*Tw(1:3,1:3);
q6adr = atan2(R6adr(1,2),R6adr(3,2));
qinv(4,6) = q6adr;
end

if ~any(any(isnan(qinv(5:6,:))))
% Calculo del wrist bd
T03bd = irb140MthSeq([1 2 3],[qinv(5,1) qinv(5,2) qinv(5,3)],l)*transl(0,l(4),0);
R46bd = T03bd(1:3,1:3)'*Tw(1:3,1:3);
q4bdl = atan2(R46bd(3,3),-R46bd(1,3));
qinv(5,4) = q4bdl;
q4bdr = atan2(-R46bd(3,3),R46bd(1,3));
qinv(6,4) = q4bdr;
% Wrist izquierdo
T04bdl = irb140MthSeq([1 2 3 4],[qinv(5,1) qinv(5,2) qinv(5,3) qinv(5,4)],l);
R56bdl = T04bdl(1:3,1:3)'*Tw(1:3,1:3);
q5bdl = atan2(-R56bdl(1,3),R56bdl(3,3));
qinv(5,5) = q5bdl;
T05bdl = irb140MthSeq([1 2 3 4 5],[qinv(5,1) qinv(5,2) qinv(5,3) qinv(5,4) qinv(5,5)],l);
R6bdl = T05bdl(1:3,1:3)'*Tw(1:3,1:3);
q6bdl = atan2(R6bdl(1,2),R6bdl(3,2));
qinv(5,6) = q6bdl;
% Wrist derecho
T04bdr = irb140MthSeq([1 2 3 4],[qinv(5,1) qinv(5,2) qinv(5,3) qinv(6,4)],l);
R56bdr = T04bdr(1:3,1:3)'*Tw(1:3,1:3);
q5bdr = atan2(-R56bdr(1,3),R56bdr(3,3));
qinv(6,5) = q5bdr;
T05bdr = irb140MthSeq([1 2 3 4 5],[qinv(5,1) qinv(5,2) qinv(5,3) qinv(6,4) qinv(6,5)],l);
R6bdr = T05bdr(1:3,1:3)'*Tw(1:3,1:3);
q6bdr = atan2(R6bdr(1,2),R6bdr(3,2));
qinv(6,6) = q6bdr;
end

if ~any(any(isnan(qinv(7:8,:))))
% Calculo del wrist bu
T03bu = irb140MthSeq([1 2 3],[qinv(7,1) qinv(7,2) qinv(7,3)],l)*transl(0,l(4),0);
R46bu = T03bu(1:3,1:3)'*Tw(1:3,1:3);
q4bul = atan2(R46bu(3,3),-R46bu(1,3));
qinv(7,4) = q4bul;
q4bur = atan2(-R46bu(3,3),R46bu(1,3));
qinv(8,4) = q4bur;
% Wrist izquierdo
T04bul = irb140MthSeq([1 2 3 4],[qinv(7,1) qinv(7,2) qinv(7,3) qinv(7,4)],l);
R56bul = T04bul(1:3,1:3)'*Tw(1:3,1:3);
q5bul = atan2(-R56bul(1,3),R56bul(3,3));
qinv(7,5) = q5bul;
T05bul = irb140MthSeq([1 2 3 4 5],[qinv(7,1) qinv(7,2) qinv(7,3) qinv(7,4) qinv(7,5)],l);
R6bul = T05bul(1:3,1:3)'*Tw(1:3,1:3);
q6bul = atan2(R6bul(1,2),R6bul(3,2));
qinv(7,6) = q6bul;
% Wrist derecho
T04bur = irb140MthSeq([1 2 3 4],[qinv(7,1) qinv(7,2) qinv(7,3) qinv(8,4)],l);
R56bur = T04bur(1:3,1:3)'*Tw(1:3,1:3);
q5bur = atan2(-R56bur(1,3),R56bur(3,3));
qinv(8,5) = q5bur;
T05bur = irb140MthSeq([1 2 3 4 5],[qinv(7,1) qinv(7,2) qinv(7,3) qinv(8,4) qinv(8,5)],l);
R6bur = T05bur(1:3,1:3)'*Tw(1:3,1:3);
q6bur = atan2(R6bur(1,2),R6bur(3,2));
qinv(8,6) = q6bur;
end

% Validacion de limites articulares
for i=1:size(qinv,2)
   qinv(:,i) = q_lim_check(qlims(i,:),qinv(:,i));
end

% Asignacion de las configuraciones para RAPID
for i=1:size(qinv,1)
   if ~any(isnan(qinv(i,:)))
      cnfgdata(i,:) = [cfn_gen(qinv(i,:)) i-1];
   else
      cnfgdata(i,:) = [NaN NaN NaN NaN];
   end
end

% Soluciones factibles
fil = 1;
for i=1:size(qinv,1)
   if ~any(isnan(qinv(i,:)))
      sol.q(fil,:) = qinv(i,:);
      sol.cnfg(fil,:) = cnfgdata(i,:);
      fil = fil + 1;
   end
end
end

function qinv_lim = q_lim_check(qlims, qinv)

if any(or(qinv < qlims(1), qinv > qlims(2)))
  for i=1:size(qinv,1)
      if or(qinv(i) < qlims(1), qinv(i) > qlims(2))
         qinv(i) = NaN;
      end
   end
end

qinv_lim = qinv;
end

function cfn = cfn_gen(qinv)
cf1 = 0;
if and(qinv(1) >= 0, qinv(1) < pi/2)
   cf1 = 0;
elseif and(qinv(1) >= pi/2, qinv(1) < pi)
   cf1 = 1;
elseif and(qinv(1) < 0, qinv(1) >= -pi/2)
   cf1 = -1;
elseif and(qinv(1) < -pi/2, qinv(1) >= -pi)
   cf1 = -2;
end

cf4 = 0;
if and(qinv(4) >= 0, qinv(4) < pi/2)
   cf4 = 0;
elseif and(qinv(4) >= pi/2, qinv(4) < pi)
   cf4 = 1;
elseif and(qinv(4) >= pi, qinv(4) < 3*pi/2)
   cf4 = 2;
elseif and(qinv(4) >= 3*pi/2, qinv(4) < 2*pi)
   cf4 = 3;   
elseif and(qinv(4) < 0, qinv(4) >= -pi/2)
   cf4 = -1;
elseif and(qinv(4) < -pi/2, qinv(4) >= -pi)
   cf4 = -2;
elseif and(qinv(4) < -pi, qinv(4) >= -3*pi/2)
   cf4 = -3;
elseif and(qinv(4) < -3*pi/2, qinv(4) >= -2*pi)
   cf4 = -4;
end

cf6 = 0;
if and(qinv(6) >= 0, qinv(6) < pi/2)
   cf6 = 0;
elseif and(qinv(6) >= pi/2, qinv(6) < pi)
   cf6 = 1;
elseif and(qinv(6) >= pi, qinv(6) < 3*pi/2)
   cf6 = 2;
elseif and(qinv(6) >= 3*pi/2, qinv(6) < 2*pi)
   cf6 = 3;   
elseif and(qinv(6) < 0, qinv(6) >= -pi/2)
   cf6 = -1;
elseif and(qinv(6) < -pi/2, qinv(6) >= -pi)
   cf6 = -2;
elseif and(qinv(6) < -pi, qinv(6) >= -3*pi/2)
   cf6 = -3;
elseif and(qinv(6) < -3*pi/2, qinv(6) >= -2*pi)
   cf6 = -4;
end

cfn = [cf1 cf4 cf6];
end

function T = irb140MthSeq(seq, q, l)
T = eye(4);
for i=1:numel(seq)
   switch seq(i)
      case 1
         Taux = [cos(q(i)) -sin(q(i)) 0 0;
                 sin(q(i))  cos(q(i)) 0 0;
                 0          0         1 l(2);
                 0          0         0 1];
      case 2
         Taux = [sin(q(i))  cos(q(i)) 0 l(1);
                 0          0         1 0;
                 cos(q(i)) -sin(q(i)) 0 0;
                 0          0         0 1];
      case 3
         Taux = [cos(q(i)) -sin(q(i)) 0 l(3);
                 sin(q(i))  cos(q(i)) 0 0;
                 0          0         1 0;
                 0          0         0 1];
      case 4
         Taux = [cos(q(i)) -sin(q(i)) 0 0;
                 0          0         1 l(4);
                -sin(q(i)) -cos(q(i)) 0 0;
                 0          0         0 1];
      case 5
         Taux = [cos(q(i)) -sin(q(i)) 0 0;
                 0          0        -1 0;
                 sin(q(i))  cos(q(i)) 0 0;
                 0          0         0 1];
      case 6
         Taux = [-cos(q(i))  sin(q(i)) 0 0;
                  0          0         1 0;
                  sin(q(i))  cos(q(i)) 0 0;
                  0          0         0 1];
      otherwise
         Taux = eye(4);
   end
   T = T*Taux;
end
end
