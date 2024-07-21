clc
clf
clear
%%
% Parametros Dimensionales
p_i = [-0.2; 0.65];   % Posicion inicial
p_f = [0.5; -0.45];   % Posicion final
R = troty(pi); % Orientacion E.F
z0 = 0.1;
Ti =  transl(p_i(1),p_i(2),z0)*R;
Tf =  transl(p_f(1),p_f(2),z0)*R;

% Robot
escala = 1e-3;
l = escala*[70 352 360 380 65];
qlims = deg2rad([-180 180; -90 110; -230 49; -200 200; -115 115; -400 400]);
L(1) = Link('revolute','alpha', 0,    'a',0,   'd',l(2),'offset', 0,   'qlim',qlims(1,:),'modified');
L(2) = Link('revolute','alpha',-pi/2, 'a',l(1),'d',0,   'offset',-pi/2,'qlim',qlims(2,:),'modified');
L(3) = Link('revolute','alpha', 0,    'a',l(3),'d',0,   'offset', 0,   'qlim',qlims(3,:),'modified');
L(4) = Link('revolute','alpha',-pi/2, 'a',0,   'd',l(4),'offset', 0,   'qlim',qlims(4,:),'modified');
L(5) = Link('revolute','alpha', pi/2, 'a',0,   'd',0,   'offset', 0,   'qlim',qlims(5,:),'modified');
L(6) = Link('revolute','alpha',-pi/2, 'a',0,   'd',0,   'offset', pi,   'qlim',qlims(6,:),'modified');
IRB140 = SerialLink(L,'name','ABB IRB140');
IRB140.tool= transl(0,0,l(5));

% Validacion cinematica inversa de puntos iniciales
q_Ti = inv_irb_140_m(Ti,IRB140);
q_Tf = inv_irb_140_m(Tf,IRB140);
if isempty(q_Ti) || isempty(q_Tf)
    msgbox('Poses iniciales fuera del espacio de trabajo');
end

% Limites de busqueda
x_min = -.2;
x_max = 1;
y_min = -1;
y_max = 1;

% Obstaculos
porc_tol = 0.15; % Engrosamiento
n_poly_obs = 2;
n_circ_obs = 1;

% Poligonos
if n_poly_obs == 0
    pol = [];
end
x_o = [0.35 0.5 0.5 0.35];
y_o = [0.3 0.3 0.65 0.65];
z_o = [z0 z0 z0 z0];
pol(1) = struct('xv',x_o,'yv',y_o,'zv',z_o,...
    'xv_p',pol_exp_tol(x_o,porc_tol),'yv_p',pol_exp_tol(y_o,porc_tol));

x_o = [0.4 0.65 0.65 0.4];
y_o = [-0.2 -0.2 0.05 0.05];
z_o = [z0 z0 z0 z0];
pol(2) = struct('xv',x_o,'yv',y_o,'zv',z_o,...
    'xv_p',pol_exp_tol(x_o,porc_tol),'yv_p',pol_exp_tol(y_o,porc_tol));

% Circulos
if n_circ_obs == 0
    circ = [];
end
pos_o = [0 0.4];
r_o = 0.1;
r_o_porc = r_o*(1+.5);
circ(1) = struct('p_o',pos_o,'r',r_o,'z_o',z0,'r_o_p',r_o_porc);

% Parametros PSO
n_dim = 2; % Numero de dimensiones
n_pun = 3; % Numero de puntos intermedios
n_adi = 0; % Evaluaciones adicionales de puntos intermedios
n_var = n_dim*n_pun; % Numero de variables
K = 100;    % Numero de iteraciones
n_exe = 1;  % Numero de ejecuciones
ind_best_exe = 1; % Mejor ejecucion
N = 30; % Poblacion
w = 0.5; % Peso Inercial
c1 = 1.5;  % Aceleracion cognitiva
c2 = c1;
chi = 0.6; % Factor de constriccion

% Parametros Stall
stall_gen = 20; % Iteraciones sin cambio en el mejor global
%stall_gen = 60;
stall_error = 1e-3; % Variacion aceptable para actualizar mejor global
stall_iter = 0;
stall_flag = 0;

% Puntos de restriccion
p_int = 10; % Puntos intermedios
p_robot = 10; % Puntos de prueba de los eslabones

% Penalizaciones
c_pen1 = 0.1; % Penalizacion zona de engrosamiento
c_pen2 = 10; % Penalizacion colision E.F
cfg_pen = 20; % Penalizacion cambios de configuracion
cfg1_pen = 1;
ws_pen = 100; % Penalizacion por salir del espacio de trabajo

% Banderas colisiones
init_col_free_pob = 1; % Poblacion inical sin colisiones con obstaculos
init_col_free_ws_pob = 0; % Poblacion inical sin colisiones con obstaculos y espacio de trabajo

link_col_en = 0; % Verificacion de colisiones eslabones

% Display
disp_en = 1;
n_fig = 1;

% Archivo
file_en = 0;

% Rapid file
rapid_en = 1;

end_flag = 0; % Bandera de terminacion
disp('Parametros iniciales cargados.')

% Evolucion poblacion
X_evo = zeros(n_var,N,K,n_exe);
%%
% Parametros archivo
if file_en
    f_name = strcat('data_',datestr(datetime('now','format','dd_MM_yyyy''_''HH_mm'),...
        'dd_MM_yyyy_HH_mm'),'.txt');
    fileID = fopen(f_name,'w');
    fprintf(fileID,'Exe  Puntos  Distancia  (x,y)\r\n');
end

tic
for a=1:n_exe
    disp(strcat('Ejecucion_',num2str(a), '_de_', num2str(n_exe)))
    
    % Poblacion inicial aleatoria
    X_c = [x_min x_max; y_min y_max];
    X = repmat(X_c(:,2)-X_c(:,1),n_pun,N).*rand(n_var,N)+repmat(X_c(:,1),n_pun,N);
    
    c_cfg = zeros(1,N); % Configuracion
    c_cfg1 = zeros(1,N);
    c_ws = zeros(1,N); % Espacio de trabajo
    
    % Poblacion inicial aleatoria sin colisiones con obstaculos
    if init_col_free_pob == 1 && init_col_free_ws_pob ~= 1
        disp('Poblacion inicial con restricciones cinematicas')
        % Puntos Intermedios
        p_TS_ant = zeros(n_dim*(n_pun+1),N,p_int);
        % Colisiones con obstaculos
        % Contabiliza la cantidad de puntos en general, no para cada
        % trayectoria entre puntos
        c_points1 = zeros(1,N); % Zona de engrosamiento
        c_points2 = zeros(1,N); % Obstaculo
        for i=1:N
            c_flag = 1;
            while c_flag == 1
                p_TS_ind = lin_n_pun_interp(1,n_dim,n_pun,p_int,p_i,p_f,X(:,i));
                % Colisiones con obstaculos
                [c_points1(i), c_points2(i)] = c_check(n_pun,p_TS_ind,...
                    n_poly_obs, pol, n_circ_obs, circ);
                if c_points2(i) > 0
                    X(:,i) = repmat(X_c(:,2)-X_c(:,1),n_pun,1).*rand(n_var,1)+repmat(X_c(:,1),n_pun,1);
                else
                    p_TS_ant(:,i,:) = p_TS_ind;
                    c_flag = 0;
                end
            end
        end
    end
    
    % Poblacion inicial aleatoria sin colisiones con obstaculos y dentro del
    % espacio de trabajo
    if init_col_free_pob == 1 && init_col_free_ws_pob == 1
        disp('Poblacion inicial con restricciones cinematicas')
        % Puntos Intermedios
        p_TS_ant = zeros(n_dim*(n_pun+1),N,p_int);
        % Colisiones con obstaculos
        c_points1 = zeros(1,N); % Zona de engrosamiento
        c_points2 = zeros(1,N); % Obstaculo
        for i=1:N
            c_flag = 1;
            while c_flag == 1
                c_ws(i) = 0;
                p_TS_ind = lin_n_pun_interp(1,n_dim,n_pun,p_int,p_i,p_f,X(:,i));
                % Colisiones con obstaculos
                [c_points1(i), c_points2(i)] = c_check(n_pun,p_TS_ind,...
                    n_poly_obs, pol, n_circ_obs, circ);
                % Validacion inversa
                for j=1:n_pun
                    Tn = transl(X(n_dim*j-1,i),X(n_dim*j,i),z0)*R;
                    q_Tn = inv_irb_140_m(Tn,IRB140);
                    if isempty(q_Tn.q)
                        c_ws(i) = c_ws(i) + 1;
                    end
                end
                %disp([num2str(c_points2(i)) '  ' num2str(c_ws(i))])
                if c_points2(i) > 0 || c_ws(i) > 0
                    X(:,i) = repmat(X_c(:,2)-X_c(:,1),n_pun,1).*rand(n_var,1)+repmat(X_c(:,1),n_pun,1);
                else
                    p_TS_ant(:,i,:) = p_TS_ind;
                    c_flag = 0;
                end
            end
        end
    end
    
    % Validacion inversa
    if init_col_free_ws_pob ~= 1
        if init_col_free_pob ~= 1
            p_TS_ant = zeros(n_dim*(n_pun+1),N,p_int);
            c_points1 = zeros(1,N); % Zona de engrosamiento
            c_points2 = zeros(1,N); % Obstaculo
        end
        for i=1:N
            if init_col_free_pob ~= 1
                p_TS_ind = lin_n_pun_interp(1,n_dim,n_pun,p_int,p_i,p_f,X(:,i));
                p_TS_ant(:,i,:) = p_TS_ind;
                % Colisiones con obstaculos
                [c_points1(i), c_points2(i)] = c_check(n_pun,p_TS_ind,...
                    n_poly_obs, pol, n_circ_obs, circ);
            end
            for j=1:n_pun
                Tn = transl(X(n_dim*j-1,i),X(n_dim*j,i),z0)*R;
                q_Tn = inv_irb_140_m(Tn,IRB140);
                if isempty(q_Tn.q)
                    c_ws(i) = c_ws(i) + 1;
                end
            end
            
        end
    end
    
    % Validaciones configuraciones
    for i=1:N
        for j=1:n_pun+1
            for k=1:p_int
                p_TS_ant_x(1,k) = p_TS_ant(2*j-1,i,k);
                p_TS_ant_y(1,k) = p_TS_ant(2*j,i,k);
                Tn = transl(p_TS_ant_x(1,k),p_TS_ant_y(1,k),z0)*R;
                q_Tn = inv_irb_140_m(Tn,IRB140);
                %disp(((j-1)*p_int)+k);
                %q_gbest(((j-1)*p_int)+k+1,:) = q_Tn.q(1,:);
                if ~isempty(q_Tn.cnfg)
                    cfg_traj(((j-1)*p_int)+k,:,i) = q_Tn.cnfg(1,4);
                    cfg1_traj(((j-1)*p_int)+k,:,i) = q_Tn.cnfg(1,1);
                else
                    cfg_traj(((j-1)*p_int)+k,:,i) = NaN;
                    cfg1_traj(((j-1)*p_int)+k,:,i) = NaN;
                end
            end
        end
        for m = 2:size(cfg_traj,1)
            if cfg_traj(m-1,:,i) ~= cfg_traj(m,:,i)
                c_cfg(i) = c_cfg(i) + 1;
            end
        end
        for m = 2:size(cfg1_traj,1)
            if (cfg1_traj(m-1,:,i) == 1 && cfg1_traj(m,:,i) == - 2) || ...
                    (cfg1_traj(m-1,:,i) == -2 && cfg1_traj(m,:,i) == 1)
                c_cfg1(i) = c_cfg1(i) + 1;
            end
        end
    end
    
    % Validaciones cfg1
    %{
   for i=1:N
      for j=1:n_pun+1
         for k=1:p_int
            p_TS_ant_x(1,k) = p_TS_ant(2*j-1,i,k);
            p_TS_ant_y(1,k) = p_TS_ant(2*j,i,k);
            Tn = transl(p_TS_ant_x(1,k),p_TS_ant_y(1,k),z0)*R;
            q_Tn = inv_irb_140_m(Tn,IRB140);
            if ~isempty(q_Tn.cnfg)
               cfg1_traj(((j-1)*p_int)+k,:,i) = q_Tn.cnfg(1,1);
            else
               cfg1_traj(((j-1)*p_int)+k,:,i) = NaN;
            end
         end
      end
      for m = 2:size(cfg1_traj,1)
         if (cfg1_traj(m-1,:,i) == 1 && cfg1_traj(m,:,i) == - 2) || ...
            (cfg1_traj(m-1,:,i) == -2 && cfg1_traj(m,:,i) == 1)
            c_cfg1(i) = c_cfg1(i) + 1;
         end
      end
   end
    %}
    
    % Vector de velocidades iniciales (aleatorio)
    V = repmat(X_c(:,2)-X_c(:,1),n_pun,N).*rand(n_var,N)-repmat(X_c(:,2)-X_c(:,1),n_pun,N);
    
    % Funcion objetivo
    F_obj = dist_2D_n_pun(N,n_dim,n_pun,p_i,p_f,X);
    F_obs = (1 + c_pen1*sum(c_points1) + c_pen2*sum(c_points2) + cfg_pen*c_cfg + ws_pen*c_ws + cfg1_pen*c_cfg1);
    F_fit = F_obj.*F_obs;
    
    % Asignar mejor individual poblacion inicial
    p_best = X;
    fp_best = F_fit; % Evaluacion funcion objetivo poblacion
    
    % Mejor punto
    [best_F, best_I] = min(F_fit);
    
    % Buscar mejor global poblacion inicial
    g_best = X(:,best_I);  % Variables
    fg_best = best_F;   % Valor Optimo
    best_c_points = c_points2(best_I);
    if a == 1
        g_best_exe = g_best;
        fg_best_exe = fg_best;
    end
    
    % Visualizacion
    if disp_en
        figure(a)
        subplot(2,1,1)
        plot(1,fg_best,'b.',1,fg_best,'k.')
        %plot(1,fg_best,'k.')
        hold on
        grid on
        axis([0 K 0 round(fg_best+1)])
        xlabel('Iteracion')
        ylabel('Valor funcion objetivo')
        title('Evolucion Fitness')
        % Leyenda para diferenciar
        %legend('Mejor Fitness','Fitnes_{avg}')
        pause(0.000001)
    end
    
    disp('Poblacion inicial encontrada.')
    
    % Ejecucion de K iteraciones
    for b=1:K
        % Muestra de procentaje de ejecucion
        if mod(100-100*(K-b)/K,10) == 0
            disp(strcat('Ejecutado al:',num2str(100-100*(K-b)/K),'%'))
        end
        stall_flag = 0;
        
        % Evolucion de la poblacion (Posiciones X, para K iteraciones)
        X_evo(:,:,b,a) = X;
        
        % Peso Inercial constante
        %w = 1;
        % Peso inercial linealmente decreciente
        w = (0.9*K-0.5*b-0.4)/(K-1);
        %disp(w)
        % Actualizacion de Velocidad
        V = w*V + ...
            c1*rand(n_var,N).*(p_best-X) + ...
            c2*rand(n_var,N).*(repmat(g_best,1,N)-X);
        
        % Limitar la velocidad
        V = max(V,repmat(X_c(:,1)-X_c(:,2),n_pun,N));
        V = min(V,repmat(X_c(:,2)-X_c(:,1),n_pun,N));
        
        % Actualizacion de Posicion
        X = X + chi*V;
        
        % Pared Absorbente
        for i=1:N
            for j=1:n_pun
                [X(n_dim*(j-1)+1:n_dim*j,i), flag] = check_x(X(n_dim*(j-1)+1:n_dim*j,i),...
                    [x_min y_min],[x_max y_max]);
                if flag
                    V(n_dim*(j-1)+1:n_dim*j,i) = 0;
                end
            end
        end
        
        % Interpolacion
        p_TS_ant = lin_n_pun_interp(N,n_dim,n_pun,p_int,p_i,p_f,X);
        
        % Distancias
        % Cartesiana espacio de la tarea
        F_obj = dist_2D_n_pun(N,n_dim,n_pun,p_i,p_f,X);
        
        c_cfg = zeros(1,N); % Configuraciones trayectoria
        c_cfg1 = zeros(1,N);
        c_ws = zeros(1,N); % Espacio de trabajo
        
        % Validacion espacio de trabajo y colisiones con obstaculos
        if link_col_en ~= 1
            c_points1 = zeros(n_pun+1,N); % Zona de engrosamiento
            c_points2 = zeros(n_pun+1,N); % Obstaculo
            for i=1:N
                % Colisiones con obstaculos
                [c_points1(1,i), c_points2(1,i)] = c_check(n_pun,p_TS_ant(:,i,:),...
                    n_poly_obs, pol, n_circ_obs, circ);
                for j=1:n_pun
                    Tn = transl(X(n_dim*j-1,i),X(n_dim*j,i),z0)*R;
                    q_Tn = inv_irb_140_m(Tn,IRB140);
                    % Validacion inversa
                    if isempty(q_Tn.q)
                        c_ws(i) = c_ws(i) + 1;
                    end
                end
                
            end
        end
        
        % Validaciones configuraciones
        for i=1:N
            for j=1:n_pun+1
                for k=1:p_int
                    p_TS_ant_x(1,k) = p_TS_ant(2*j-1,i,k);
                    p_TS_ant_y(1,k) = p_TS_ant(2*j,i,k);
                    Tn = transl(p_TS_ant_x(1,k),p_TS_ant_y(1,k),z0)*R;
                    q_Tn = inv_irb_140_m(Tn,IRB140);
                    if ~isempty(q_Tn.cnfg)
                        cfg_traj(((j-1)*p_int)+k,:,i) = q_Tn.cnfg(1,4);
                        cfg1_traj(((j-1)*p_int)+k,:,i) = q_Tn.cnfg(1,1);
                    else
                        cfg_traj(((j-1)*p_int)+k,:,i) = NaN;
                        cfg1_traj(((j-1)*p_int)+k,:,i) = NaN;
                    end
                end
            end
            for m = 2:size(cfg_traj,1)
                if cfg_traj(m-1,:,i) ~= cfg_traj(m,:,i)
                    c_cfg(i) = c_cfg(i) + 1;
                end
            end
            for m = 2:size(cfg1_traj,1)
                if (cfg1_traj(m-1,:,i) == 1 && cfg1_traj(m,:,i) == - 2) || ...
                        (cfg1_traj(m-1,:,i) == -2 && cfg1_traj(m,:,i) == 1)
                    c_cfg1(i) = c_cfg1(i) + 1;
                end
            end
        end
        
        % Validaciones cfg1
        %{        
        for i=1:N
            for j=1:n_pun+1
                for k=1:p_int
                    p_TS_ant_x(1,k) = p_TS_ant(2*j-1,i,k);
                    p_TS_ant_y(1,k) = p_TS_ant(2*j,i,k);
                    Tn = transl(p_TS_ant_x(1,k),p_TS_ant_y(1,k),z0)*R;
                    q_Tn = inv_irb_140_m(Tn,IRB140);
                    if ~isempty(q_Tn.cnfg)
                        cfg1_traj(((j-1)*p_int)+k,:,i) = q_Tn.cnfg(1,1);
                    else
                        cfg1_traj(((j-1)*p_int)+k,:,i) = NaN;
                    end
                end
            end
            for m = 2:size(cfg1_traj,1)
                if (cfg1_traj(m-1,:,i) == 1 && cfg1_traj(m,:,i) == - 2) || ...
                        (cfg1_traj(m-1,:,i) == -2 && cfg1_traj(m,:,i) == 1)
                    c_cfg1(i) = c_cfg1(i) + 1;
                end
            end
        end
        %}
        
        % Evaluacion funcion objetivo
        % Penalizacion de colisiones
        F_obs = (1 + c_pen1*sum(c_points1) + c_pen2*sum(c_points2) + cfg_pen*c_cfg + ws_pen*c_ws + cfg1_pen*c_cfg1);
        % Evalaucion de fitness (Objetivo + Penalizaciones)
        F_fit = F_obj.*F_obs;
        
        % Busqueda de mejor fitness
        [best_F, best_I] = min(F_fit);
        p_best_k = X(:,best_I);
        
        % Actualizacion de los mejores individuales
        for j=1:N
            % Actualizacion de mejores posiciones inviduales
            if F_fit(j) < fp_best(j)
                p_best(:,j) = X(:,j);
                fp_best(j) = F_fit(j);
            end
        end
        
        % Actualizacion del  mejor global
        % Comparacion con el mejor global almacenado
        if best_F < fg_best
            % Actualizacion de mejor global
            g_best = p_best_k;
            if abs(fg_best-best_F) >= stall_error
                stall_flag = 1;
            end
            fg_best = best_F;
            best_c_points = sum(c_points2(:,best_I));
            %best_c_joints = c_joints(best_I);
            
        end
        
        fg_evo(a,b) = fg_best; % Historico mejor valor global por cada iteracion
        
        % Revision de Stall
        if stall_flag
            stall_iter = 0;
        else
            stall_iter = stall_iter + 1;
        end
        if  stall_iter >= stall_gen
            end_flag = 1;
            disp('Terminacion por criterio de Stall.')
            stall_iter = 0;
            break;
        end
        
        % Visualizacion
        if disp_en
            figure(a)
            if b > 1
                subplot(2,1,1)
                plot([b-1 b],[fg_evo(a,b-1) fg_evo(a,b)],'b','linewidth',2)
                
            end
            subplot(2,1,1)
            %plot(b,fg_best,'b*')
            plot(b,mean(fg_evo(a,1:b)),'k.')
            pause(0.000001)
            
            subplot(2,1,2)
            cla
            bar =[round(100*stall_iter/stall_gen), 100-100*(K-b)/K, 0];
            barh(bar,'BarWidth',0.2);
            axis([0 100 0 3]);
            xlabel('Porcentaje (%)')
            title('Cumplimiento Stall')
            pause(0.000001)
        end
        
    end
    
    % Mejor global en todas las ejecuciones
    if fg_best < fg_best_exe
        g_best_exe = g_best;
        fg_best_exe = fg_best;
        ind_best_exe = a;
        best_c_points_exe = best_c_points;
        if best_c_points_exe > 0
            end_flag = 2;
        end
    end
    
    disp(strcat('Finalizacion ejecucion_',num2str(a),'_de_',num2str(n_exe)))
    %{
       while 1
           str_in = input('Desea continuar (Y)','s');
           if strcmp(str_in,'Y')
               break
           end
       end
    %}
    
    if file_en
        % Guardar datos
        fprintf(fileID,'%1.0f \t',a);
        fprintf(fileID,'%1.0f \t',n_pun);
        fprintf(fileID,'%1.4f \t',fg_best);
        for c=1:n_pun
            fprintf(fileID,'(%1.3f, %1.3f) \t',[g_best(2*c-1), g_best(2*c)]);
            if c == n_pun
                fprintf(fileID,'\r\n');
            end
        end
        %fclose(fileID);
    end
    
end

stall_ind = b; % Indice de stall

toc;
t_time = toc; % Tiempo de la ejecuciones

% Cierre de archivo
if file_en
    fclose(fileID);
end
%%
% Generacion de codigo en RAPID
if rapid_en
    %q_Ti = inv_irb_140_m(Ti,IRB140);
    %q_Tf = inv_irb_140_m(Tf,IRB140);
    for i=1:n_pun
        if i == 1
            Taux = transl(g_best_exe(2*i-1),g_best_exe(2*i),z0)*R;
            T = cat(3,Ti,Taux);
            q_Ti = inv_irb_140_m(Ti,IRB140);
            q_Taux = inv_irb_140_m(Taux,IRB140);
            cfg = cat(3,q_Ti.cnfg(1,:),q_Taux.cnfg(1,:));
        end
        if i > 1
            Taux = transl(g_best_exe(2*i-1),g_best_exe(2*i),z0)*R;
            T = cat(3,T,Taux);
            q_Taux = inv_irb_140_m(Taux,IRB140);
            cfg = cat(3,cfg,q_Taux.cnfg(1,:));
        end
    end
    T = cat(3,T,Tf);
    T(1:3,4,:)=1000*T(1:3,4,:); % Ajuste de m a mm para los robtargets
    cfg = cat(3,cfg,q_Tf.cnfg(1,:));
    gen_rapid(T,cfg,'modulo_test15_v1')
end
%%
% Grafica del entorno con obstaculos
if disp_en
    figure(a+1)
    hold on
    grid on
    plot(p_i(1),p_i(2),'ko')
    plot(p_f(1),p_f(2),'ko')
    axis([-1 1 -1 1])
    % Obstaculos
    % Circulos
    for jj=1:n_circ_obs
        rectangle('Position',[circ(jj).p_o(1)-circ(jj).r_o_p circ(jj).p_o(2)-circ(jj).r_o_p 2*circ(jj).r_o_p...
            2*circ(jj).r_o_p],'Curvature',1,'FaceColor',[0.76 1 0.83],'EdgeColor','g')
        rectangle('Position',[circ(jj).p_o(1)-circ(jj).r circ(jj).p_o(2)-circ(jj).r 2*circ(jj).r 2*circ(jj).r],...
            'Curvature',1,'FaceColor',[1 0.76 0.76],'EdgeColor','r')
    end
    % Poligonos
    for jj=1:n_poly_obs
        patch(pol(jj).xv_p,pol(jj).yv_p,[0.76 1 0.83],'EdgeColor','g')
        patch(pol(jj).xv,pol(jj).yv,[1 0.76 0.76],'EdgeColor','r')
    end
end
%%
% Mejor trayectoria
if disp_en
    p_TS_ant_gbest = lin_n_pun_interp(1,n_dim,n_pun,p_int,p_i,p_f,g_best_exe);
    for j=1:n_pun+1
        for k=1:p_int
            p_TS_ant_x(1,k) = p_TS_ant_gbest(2*j-1,1,k);
            p_TS_ant_y(1,k) = p_TS_ant_gbest(2*j,1,k);
        end
        if j < n_pun+1
            plot(g_best_exe(2*j-1,1),g_best_exe(2*j,1),'k*');
        end
        plot(p_TS_ant_x(:),p_TS_ant_y(:),'x')
    end
end
%%
% Calculo de valores articulares y configuraciones
%Ti =  transl(p_i(1),p_i(2),z0)*troty(pi);
%q_Ti = inv_irb_140_m(Ti,IRB140);
q_gbest(1,:) = q_Ti.q(1,:);
cfg_gbest(1,:) = q_Ti.cnfg(1,4);
for j=1:n_pun+1
    for k=1:p_int
        p_TS_ant_x(1,k) = p_TS_ant_gbest(2*j-1,1,k);
        p_TS_ant_y(1,k) = p_TS_ant_gbest(2*j,1,k);
        Tn = transl(p_TS_ant_x(1,k),p_TS_ant_y(1,k),z0)*troty(pi);
        q_Tn = inv_irb_140_m(Tn,IRB140);
        q_gbest(((j-1)*p_int)+k+1,:) = q_Tn.q(1,:);
        cfg_gbest(((j-1)*p_int)+k+1,:) = q_Tn.cnfg(1,4);
    end
end
%%
% Graficas de q's
if disp_en
    figure(a+2)
    plot(0:size(q_gbest,1)-1,q_gbest,'lineWidth',1.5)
    grid on
    xlabel('Punto de la trayectoria','Color',[0 0 0], 'fontSize',11,'fontWeight','bold')
    ylabel('Posicion angular (rad)','Color',[0 0 0], 'fontSize',11,'fontWeight','bold')
    legend('q_1','q_2','q_3','q_4','q_5','q_6')
    set(gca,'fontSize',11)
    set(gcf,'units','points','position',[0,0,500,500])
    axis([0 size(q_gbest,1)-1 -4 4])
end
%%
% Graficas de la configuracion
if disp_en
    figure(a+3)
    stem(0:size(cfg_gbest,1)-1,cfg_gbest)
    grid on
    xlabel('Punto de la trayectoria','Color',[0 0 0], 'fontSize',11,'fontWeight','bold')
    ylabel('Configuracion del manupualdor (cfgx)','Color',[0 0 0], 'fontSize',11,'fontWeight','bold')
    set(gca,'fontSize',11)
    set(gcf,'units','points','position',[0,0,500,500])
    axis([0 size(q_gbest,1)-1 0 7])
end
%%
% Evolucion de las soluciones
% Unicamente considera la mejor ejecucion
if disp_en
    col = rand(N,3); % Color aleatorio para la grafica
    for ii=40:stall_ind % Ciclo de iteracion
        for jj=1:1 % Particula
            for kk=1:n_pun % Cantidad de puntos de la solucion
                X_ax(kk,:) = X_evo(2*kk-1,jj,ii,ind_best_exe);
                X_ay(kk,:) = X_evo(2*kk,jj,ii,ind_best_exe);
                if ii < stall_ind
                    X_nx(kk,:) = X_evo(2*kk-1,jj,ii+1,ind_best_exe);
                    X_ny(kk,:) = X_evo(2*kk,jj,ii+1,ind_best_exe);
                end
                plot(X_ax(kk,:),X_ay(kk,:),'*','color',col(jj,:))
                plot(X_nx(kk,:),X_ny(kk,:),'*','color',col(jj,:))
                text(X_ax(kk,:)+0.03,X_ay(kk,:),num2str(ii),'color',col(jj,:))
                plot([X_ax(kk,:) X_nx(kk,:)],[X_ay(kk,:) X_ny(kk,:)],'-','color',col(jj,:))
            end
        end
    end
end
%%
% Valor f_fit
if disp_en
    figure(a+4)
    hold on
    grid on
    for ii=1:n_exe
        plot(fg_evo(ii,:))
    end
end