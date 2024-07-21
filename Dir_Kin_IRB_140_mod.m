%% Cinematica directa AAB IRB 140
% DHmod
escala = 0.01;
l = escala*[70 352 360 380 65];
qlims = deg2rad([-180 180; -90 110; -230 50; -200 200; -115 115; -400 400]);
L(1) = Link('revolute','alpha', 0,    'a',0,   'd',l(2),'offset', 0,   'qlim',qlims(1,:),'modified');
L(2) = Link('revolute','alpha',-pi/2, 'a',l(1),'d',0,   'offset',-pi/2,'qlim',qlims(2,:),'modified');
L(3) = Link('revolute','alpha', 0,    'a',l(3),'d',0,   'offset', 0,   'qlim',qlims(3,:),'modified');
L(4) = Link('revolute','alpha',-pi/2, 'a',0,   'd',l(4),'offset', 0,   'qlim',qlims(4,:),'modified');
L(5) = Link('revolute','alpha', pi/2, 'a',0,   'd',0,   'offset', pi,  'qlim',qlims(5,:),'modified');
L(6) = Link('revolute','alpha', pi/2, 'a',0,   'd',0,   'offset', 0,   'qlim',qlims(6,:),'modified');
L(7) = Link('revolute','alpha',0,     'a',0,   'd',l(5),'offset', 0,   'qlim',[]        ,'modified');

IRB140 = SerialLink(L,'name','ABB IRB140');
%%
trplot(eye(4),'length',2,'rgb');
hold on
IRB140.plot([0 0 0 0 pi/6 0 0],'workspace',escala*[-1000 1000 -1000 1000 -1000 1000],...
            'scale',0.4,'jaxes');
axis(escala*[-1000 1000 -1000 1000 -1000 1000]);
IRB140.teach()
%%
% Plantear dir kin DHstd - ok
% Resolver inverse kinematics (probar ikine,ikunc,code sofia)
% Probar generacion de trayectorias multipunto
%%
syms q1 q2 q3 q4 q5 q6 as real
%%
A01 = simplify(L(1).A(q1));
A01(isAlways(A01 <= eps,'Unknown','false') ) = 0;
%%
A12 = simplify(L(2).A(q2));
A12( isAlways(A12 <= eps,'Unknown','false') ) = 0;
%%
A23 = simplify(L(3).A(q3));
A23( isAlways(A23 <= eps,'Unknown','false') ) = 0;
%%
A34 = simplify(L(4).A(q4));
A34( isAlways(A34 <= 1e-10,'Unknown','false') ) = 0;
%%
A45 = simplify(L(5).A(q5));
A45( isAlways(A34 <= 1e-10,'Unknown','false') ) = 0;
%%
A56 = simplify(L(6).A(q6));
A56( isAlways(A56 <= 1e-10,'Unknown','false') ) = 0;
%%
A67 = L(7).A(0);
A67( isAlways(A67 <= 1e-10,'Unknown','false') ) = 0;
%%
A07 = simplify(A01*A12*A23*A34*A45*A56*A67);
%%
q_test = pi*[1/4 1/6 -1/6 1/4 1/6 1/12 0];
T_test = IRB140.fkine(q_test);
%%
[q,err,exitflag] = IRB140.ikunc(T_test, zeros(1,7))