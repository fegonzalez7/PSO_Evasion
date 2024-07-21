%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Determina si existe interferencia entre un obstaculo y el efetor final
% con un perfil de movimiento en el espacio de la tarea
% Entradas:
% n_pun = Cantidad de puntos intermedios
% p_TS_ant = Puntos de la trayectoria a verificar
% n_poly_obs = Cantidad de obstaculos poligonales
% pol = Estructura que contiene los poligonos
% n_circ_obs = Cantidad de obstaculos circulares
% circ = Estructura que contiene los circulos
% Salidas:
% c_points = Cantidad de puntos en los que interfiere el perfil con el
% obstaculo
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [C_points1, C_points2] = c_check(n_pun, p_TS_ant,...
    n_poly_obs, pol, n_circ_obs, circ)
C_points1 = 0; % Zona de engrosamiento
C_points2 = 0; % Obstaculo
for j=1:n_pun+1
    xp(1,:) = p_TS_ant(2*j-1,1,:);
    yp(1,:) = p_TS_ant(2*j,1,:);
    for k=1:n_poly_obs
        in = inpolygon(xp,yp,pol(k).xv_p,pol(k).yv_p);
        C_points1 = C_points1 + sum(in);
        in = inpolygon(xp,yp,pol(k).xv,pol(k).yv);
        C_points2 = C_points2 + sum(in);
    end
    for k=1:n_circ_obs
        C_points1 = C_points1 +...
            obs_check_cir(circ(k).p_o,circ(k).r_o_p,[xp; yp]);
        C_points2 = C_points2 +...
            obs_check_cir(circ(k).p_o,circ(k).r,[xp; yp]);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Determina si existe interferencia entre un obstaculo circular y el efetor 
% final con un perfil de movimiento en el espacio de la tarea
% Entradas:
% pos_o = Origen del circulo
% r_o = Radio del circulo
% p_TS = Puntos de la trayectoria a verificar
% Salidas:
% c_points = Cantidad de puntos en los que interfiere el perfil con el
% obstaculo
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function c_points = obs_check_cir(pos_o, r_o,p_TS)
c_points = 0;
if length(size(p_TS)) > 2
   switch size(p_TS,3)
      case 0
         warning('Zero vector')
      case 2
         disp('Size 2')
         for i=1:size(p_TS,2)
            if ( pdist([p_TS(1,i) p_TS(2,i); pos_o(1) pos_o(2)]) <= r_o)
               c_points = c_points + 1;
            end
         end
      case 3
         disp('Size 3')
         for i=1:size(p_TS,3)
            if ( pdist([p_TS(1,1,i) p_TS(2,1,i); pos_o(1) pos_o(2)]) <= r_o)
               c_points = c_points + 1;
            end
         end
      otherwise
         disp('Wrong size')
   end
else
   for i=1:size(p_TS,2)
      if ( pdist([p_TS(1,i) p_TS(2,i); pos_o(1) pos_o(2)]) <= r_o)
         c_points = c_points + 1;
      end
   end
end