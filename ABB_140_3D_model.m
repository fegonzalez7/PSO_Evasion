%% Cinematica directa AAB IRB 140
% DHstd
escala = 0.001;
l = escala*[70 352 360 380 65];
qlims = deg2rad([-180 180; -90 110; -230 50; -200 200; -115 115; -400 400]);
L(1) = Link('revolute','alpha',-pi/2, 'a',l(1),'d',l(2),'offset', 0,   'qlim',qlims(1,:));
L(2) = Link('revolute','alpha', 0   , 'a',l(3),'d',0,   'offset',-pi/2,'qlim',qlims(2,:));
L(3) = Link('revolute','alpha',-pi/2, 'a',0   ,'d',0,   'offset', 0,   'qlim',qlims(3,:));
L(4) = Link('revolute','alpha', pi/2, 'a',0,   'd',l(4),'offset', 0,   'qlim',qlims(4,:));
L(5) = Link('revolute','alpha', pi/2, 'a',0,   'd',0,   'offset', pi,  'qlim',qlims(5,:));
L(6) = Link('revolute','alpha',    0, 'a',0,   'd',l(5),'offset', 0,   'qlim',qlims(6,:));
IRB140 = SerialLink(L,'name','ABB IRB140');
%%
qh = [0 0 0 0 pi/6 0];
IRB140.model3d = 'ABB/IRB140';
IRB140.plot3d(qh)
IRB140.teach()

%%
% cellC = {};
% for i=1:6
%     cellC{i,1} = [1 .55 0]; 
% end
%%
C = repmat([1 .55 0],6,1);
IRB140.plot3d(qh,'notiles','color',C)
%IRB140.plot3d(qh,'notiles','color',cellC)
%%
hold on
%trplot(eye(4),'length',1,'rgb','frame','0','arrow');
axis(escala*[-1000 1000 -1000 1000 0 1000]);
xlabel('x (m)','Color',[0 0 0], 'fontSize',11,'fontWeight','bold')
ylabel('y (m)','Color',[0 0 0], 'fontSize',11,'fontWeight','bold')
zlabel('z (m)','Color',[0 0 0], 'fontSize',11,'fontWeight','bold')
set(gca,'fontSize',11)
set(gcf,'units','points','position',[0,0,500,500])
%v = [[0 0 0]' [1 0 0]' [1 0 1]' [0 0 1]' [0 1 0]' [1 1 0]' [1 1 1]' [0 1 1]']; 
%v2D = [[0 0]' [1 0]' [1 1]' [0 1]'];
%p = Polygon(v2D);
%figure
% Generar cubo 
plotcube([0.2 0.4 0.2],[0.3 0.3 0.2],0.7,[1 0 0]);
hold on
grid on
% Generar elipsoide
[elli_x, elli_y, elli_z] = ellipsoid(0.4,-0.4,0.3,0.2,0.2,0.2,25);
surf(elli_x,elli_y,elli_z,'EdgeColor',[0 0 0],'EdgeAlpha',0.75,...
    'FaceColor',[1 0 0],'FaceAlpha',0.7)
%axis(escala*[-1000 1000 -1000 1000 0 1000]);
v1 = [1 1 0.3; -1 1 0.3; -1 -1 0.3; 1 -1 0.3];
f1 = [1 2 3 4];
patch('Faces',f1,'Vertices',v1,'FaceColor',[1 0.6 0.75],'FaceAlpha',0.25)
plot3(0.1,-0.5,0.3,'k*')
text(0.05,-0.63,0.3,'Punto Inicial','FontSize',12,'FontWeight','bold')
plot3(0.6,0.5,0.3,'k*')
text(0.65,0.38,0.3,'Punto Final','FontSize',12,'FontWeight','bold')
view(90,90)
%%
figure
rectangle('Position',[0.3 0.3 0.2 0.4],'Curvature',0,'FaceColor',[1 0.76 0.76],'EdgeColor','r')
hold on 
grid on
rectangle('Position',[0.2 -0.6 0.4 0.4],'Curvature',1,'FaceColor',[1 0.76 0.76],'EdgeColor','r')
axis([-1 1 -1 1])
xlabel('x (m)','Color',[0 0 0], 'fontSize',11,'fontWeight','bold')
ylabel('y (m)','Color',[0 0 0], 'fontSize',11,'fontWeight','bold')
plot3(0.1,-0.5,0.3,'k*')
text(0.05,-0.63,0,'Punto Inicial','FontSize',12,'FontWeight','bold')
plot3(0.6,0.5,0.3,'k*')
text(0.65,0.38,0,'Punto Final','FontSize',12,'FontWeight','bold')
set(gca,'fontSize',11)
set(gcf,'units','points','position',[0,0,500,500])
view(90,90)
%%
figure
rectangle('Position',[0.25 0.25 0.3 0.5],'Curvature',0,'FaceColor',[0.76 1 0.83],'EdgeColor','g')
hold on 
grid on
rectangle('Position',[0.3 0.3 0.2 0.4],'Curvature',0,'FaceColor',[1 0.76 0.76],'EdgeColor','r')
rectangle('Position',[0.175 -0.625 0.45 0.45],'Curvature',1,'FaceColor',[0.76 1 0.83],'EdgeColor','g')
rectangle('Position',[0.2 -0.6 0.4 0.4],'Curvature',1,'FaceColor',[1 0.76 0.76],'EdgeColor','r')
axis([-.5 1 -1 1])
xlabel('x (m)','Color',[0 0 0], 'fontSize',11,'fontWeight','bold')
ylabel('y (m)','Color',[0 0 0], 'fontSize',11,'fontWeight','bold')
set(gca,'fontSize',11)
%set(gcf,'units','points','position',[0,0,500,500])
view(90,90)
