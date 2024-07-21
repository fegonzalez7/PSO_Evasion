%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% Entradas:
% 
% Salidas:
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function p_TS_ant = lin_n_pun_interp(N,n_dim,n_pun,p_int,p_i,p_f,X)
p_TS_ant = zeros(n_dim*(n_pun+1),N,p_int);
for i=1:N
    if n_pun == 1
        p_TS_ant(1:n_dim,i,:) = lin_point_interp(n_dim,p_int,p_i,X(:,i));
        p_TS_ant(n_pun*n_dim+1:(n_pun+1)*n_dim,i,:) =...
            lin_point_interp(n_dim,p_int,X(:,i),p_f);
    else
        for j=1:n_pun+1
            if j == 1
                p_TS_ant(1:n_dim,i,:) =...
                    lin_point_interp(n_dim,p_int,p_i,X(1:n_dim,i));
            end
            if j > 1 && j < n_pun+1
                p_TS_ant(n_dim*(j-1)+1:n_dim*j,i,:) =...
                    lin_point_interp(n_dim,p_int,X(n_dim*(j-2)+1:n_dim*(j-1),i),X(n_dim*(j-1)+1:n_dim*(j),i));
            end
            if j == n_pun+1
                p_TS_ant(n_dim*(j-1)+1:n_dim*j,i,:) =...
                    lin_point_interp(n_dim,p_int,X(n_dim*(j-2)+1:n_dim*(j-1),i),p_f);
            end
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% Entradas:
% 
% Salidas:
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function p_TS_ant = lin_point_interp(n_var,p_interp,p_init,p_fin)
p_TS_ant = zeros(n_var,1,p_interp);
for j=1:p_interp
    dist_x = p_fin(1)-p_init(1);
    dist_y = p_fin(2)-p_init(2);
    if dist_x >= 0
        p_TS_ant(1,1,j) = p_init(1)+j*abs(dist_x/(p_interp+1));
    else
        p_TS_ant(1,1,j) = p_init(1)-j*abs(dist_x/(p_interp+1));
    end
    if dist_y >= 0
        p_TS_ant(2,1,j) = p_init(2)+j*abs(dist_y/(p_interp+1));
    else
        p_TS_ant(2,1,j) = p_init(2)-j*abs(dist_y/(p_interp+1));
    end
end
