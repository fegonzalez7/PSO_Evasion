%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Limita el vector posicion del PSO
% Entradas:
% p_k = Vector con el punto a analizar
% p_i = Vector con los limites en x
% p_f = Vector con los limites en y
% Salidas:
% p = Vector con el punto ajustado a los limites
% flag = Bandera que indica si el punto se pone en la pared absorbente
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function  [p, flag] = check_x(p_k,p_i,p_f)
x_i = p_i(1);
y_i = p_i(2);
x_f = p_f(1);
y_f = p_f(2);
p = p_k;
flag = 0;

if x_i >= x_f
    x_max = x_i;
    x_min = x_f;
else
    x_max = x_f;
    x_min = x_i;
end

if y_i >= y_f
    y_max = y_i;
    y_min = y_f;
else
    y_max = y_f;
    y_min = y_i;
end

if (p(1) > x_max || p(1) < x_min || p(2) > y_max || p(2) < y_min)
    flag = 1;
end

if p(1) > x_max
    p(1) = x_max;
elseif p(1) < x_min
    p(1) = x_min;
end

if p(2) > y_max
    p(2) = y_max;
elseif p(2) < y_min
    p(2) = y_min;
end

end