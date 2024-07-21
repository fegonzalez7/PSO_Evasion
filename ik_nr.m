%% 
% Prueba Jacob
l1 = 1;
l2 = 1;
L(1) = Link('revolute','alpha',0,'a',0,'d',0,'modified');
L(2) = Link('revolute','alpha',0,'a',1,'d',0,'modified');
RR = SerialLink(L,'name','2R');
RR.tool = transl(1,0,0);
%%
clc
qd = [pi/8 -pi/12];
xd = RR.fkine(qd);
q0 = [0 0.1]';
tic
i = 1;
while true
   Jq0 = RR.jacob0(q0);
   Jq0 = Jq0(1:2,:);
   %Jq0 = RRjacob(q0,l1,l2);
   x0 = RR.fkine(q0);
   x0 = x0(1:2,4);
   %x0 = [l1*cos(q0(1))+l2*cos(sum(q0)) l1*sin(q0(1))+l2*sin(sum(q0))]';
   % Newton
   %qn = q0 + Jq0\(xd(1:2,4)-x0);
   % Gradiente
   qn = q0 + 0.4*Jq0'*(xd(1:2,4)-x0);
   if norm(xd(1:2,4)-x0) < 1e-5
      disp('a')
      break
   end
   if norm(abs(qd'-q0)) < 1e-6
      disp('b')
      break
   end
   if abs(det(Jq0)) < 1e-4
      disp('c')
      break
   end
   q0 = qn;
   i = i+1;
end
toc
disp(q0')
disp(qd)
%%
tic
RR.ikunc(RR.fkine(qd),[0 0])
toc