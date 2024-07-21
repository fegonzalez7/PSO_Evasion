%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calcula la distancia cartesiana entre conjuntos de puntos en forma de
% vectores de trayectorias
% Entradas:
% N = Numero de trayectorias
% n_dim = Cantidad de dimensiones
% n_pun = Cantidad de puntos intermedios
% p_i = Punto inicial de la trayectoria
% p_f = Punto final de la trayectoria
% X = Vector con las trayectorias
% Salidas:
% dist_cart = Vector con la distancia de una trayectoria de multiples
% puntos
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function dist_cart = dist_2D_n_pun(N,n_dim,n_pun,p_i,p_f,X)
dist_cart = zeros(1,N);
for i=1:N
    for j=1:n_pun+1
        if j == 1
            %[p_i X(1:n_dim,i)]';
            dist_cart(i) = pdist([p_i X(1:n_dim,i)]');
            
        end
        if j > 1 && j < n_pun+1
            %[X(n_dim*(j-2)+1:n_dim*(j-1),i) X(n_dim*(j-1)+1:n_dim*(j),i)]';
            dist_cart(i) = dist_cart(i) + ...
                pdist([X(n_dim*(j-2)+1:n_dim*(j-1),i) X(n_dim*(j-1)+1:n_dim*(j),i)]');
        end
        if j == n_pun+1
            %[X(n_dim*(j-2)+1:n_dim*(j-1),i) p_f]';
            dist_cart(i) = dist_cart(i) + ...
                pdist([X(n_dim*(j-2)+1:n_dim*(j-1),i) p_f]');
        end
    end
end