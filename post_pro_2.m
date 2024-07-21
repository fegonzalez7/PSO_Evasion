% Robot
escala = 1e-3;
l = escala*[70 0 360 380 65];
qlims = deg2rad([-180 180; -90 110; -230 49; -200 200; -115 115; -400 400]);
L(1) = Link('revolute','alpha', 0,    'a',0,   'd',l(2),'offset', 0,   'qlim',qlims(1,:),'modified');
L(2) = Link('revolute','alpha',-pi/2, 'a',l(1),'d',0,   'offset',-pi/2,'qlim',qlims(2,:),'modified');
L(3) = Link('revolute','alpha', 0,    'a',l(3),'d',0,   'offset', 0,   'qlim',qlims(3,:),'modified');
L(4) = Link('revolute','alpha',-pi/2, 'a',0,   'd',l(4),'offset', 0,   'qlim',qlims(4,:),'modified');
L(5) = Link('revolute','alpha', pi/2, 'a',0,   'd',0,   'offset', 0,   'qlim',qlims(5,:),'modified');
L(6) = Link('revolute','alpha',-pi/2, 'a',0,   'd',0,   'offset', pi,   'qlim',qlims(6,:),'modified');
IRB140 = SerialLink(L,'name','ABB IRB140');
IRB140.tool= transl(0,0,l(5));
%%
Ti =  transl(.5,-.15,0.1)*troty(pi);
Posw =  Ti(1:3,4) - l(5)*Ti(1:3,3);
%%
T1 = transl(.5,-.3,0.3)*troty(pi);
Tf =  transl(.4,-.4,0.3)*troty(pi);
%%
tic 
q_Ti = inv_irb_140_m(Ti,IRB140);
q_Ti.q
toc
%%
q_Ti = inv_irb_140_m2(Ti,qlims,l);
q_Ti.q
%%
figure
IRB140.plot(q_Ti.q(1,:),'notiles')
IRB140.teach()
%%
q_T1 = inv_irb_140_m(T1,IRB140);
q_Tf = inv_irb_140_m(Tf,IRB140);
%%
% Array de MTH con way points
T = cat(3,Ti,T1,Tf);
T(1:3,4,:)=1000*T(1:3,4,:);
% Configuraciones del robot para cada pose
cfg = cat(3,q_Ti.cnfg(1,:),q_T1.cnfg(1,:),q_Tf.cnfg(1,:));
%%
gen_rapid2(T,cfg,'modTest',[200 100],0,'tool1') 
%%
pos = transl(.56,.11,.67);
ori = troty(pi/2);
T_test = pos*ori;
%%
q_T_test = inv_irb_140_m(T_test,IRB140);