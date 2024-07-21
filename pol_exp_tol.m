%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Establece valores de tolerancia para la estrucutra de obstaculos
% poligonales
% Entradas:
% x_o = Dimensiones obstaculo
% porc_tol = Porcentaje de tolerancia de la zona de engrosamiento
% Salidas:
% x_op = Valores del poligono con la zona de engrosamiento incluida
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function x_op = pol_exp_tol(x_o,porc_tol,varargin)
if nargin == 1
    porc_tol = 0.05;
    warning('Valor de tolerancia por defecto 5%')
end
d_tol = abs(max(x_o)-min(x_o))*porc_tol;
x_op = zeros(1,length(x_o));
for i=1:length(x_o)
    if x_o(i) > mean(x_o)
        x_op(i) = x_o(i) + d_tol;
    else
        x_op(i) = x_o(i) - d_tol;
    end
end