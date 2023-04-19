% Compare the expected and sensed position at time k
err_pos(:,k) = X0(1:3,i) - pred_X0(1:3,i);
err_vel(:,k) = X0(4:6,i) - pred_X0(4:6,i);

% Compare the current position and the reference
err_pos_ref(:,k) = X0(1:3,i) - X0_ref(:,1,i);
err_vel_ref(:,k) = X0(4:6,i) - X0_ref(:,2,i);

der_err_ref(:,k) = (err_pos_ref(:,k) - err_pos_ref(:,k-1)) / h;

% Cost that determines whether there's something disturbing the agent
% Cost gets higher when the error between the reference and the state gets higher
cost(:,k,i) = (err_pos_ref(:,k).^5) ./ (-X0(4:6,i)+sign(X0(4:6,i))*0.01);

% Integral term on position
integ_err(:,k) = integ_err(:,k-1) + err_pos_ref(:,k)*h;

for n = 1:ndim
    % Filter noise on position for feedback
    if abs(err_pos(n,k)) < err_tol_pos
        err_pos(n,k) = 0;
    end
end

trigger(k,i) = 0;

% Reset reference to state if the error grows large
if any(cost(:,k,i) > max_cost) || any(cost(:,k,i) < min_cost)
    X0_ref(:,1,i) = X0(1:3,i);
    X0_ref(:,2,i) = X0(4:6,i);
    X0_ref(:,3:5,i) = zeros(3,3);
    trigger(k,i) = 1;
end

% Include on-demand collision avoidance
if ~drop_comms || comm_recovery == RecoveryType.on_dem
    [A_coll, b_coll, pf_tmp, t_build(k,i)] = ondemand_softconstraints_ref(hor_ref(:,:,:,k-1), Phi_ref,...
                                                                         X0(:,i), i, rmin,...
                                                                         order, E1, E2);

    if ~isempty(b_coll) % collisions in the horizon
        % Include collision constraints and slack variables
        N_v = length(b_coll) / 3;
        A_in_i = [A_in zeros(size(A_in,1), N_v) ; A_coll];
        b_in_i = [b_in; b_coll];
        A_eq_i = [A_eq zeros(size(A_eq,1), N_v)];

        % Linear and quadratic term to penalize collision relaxation
        f_eps = lin_coll_penalty*ones(1, N_v);
        H_eps = quad_coll_penalty*eye(N_v);


        H_i = [H_o zeros(size(H_f,1), N_v);
               zeros(N_v,size(H_f,2)) H_eps];
        mat_f_x0_i = mat_f_x0_obs;
        if false && drop_comms
            f_tot = reshape(hor_ref(:,:,i,k-1), 1, [])*S_obs*Phi;
        else
            f_tot = f_pf_obs(:,:,i);
        end
    else % no collisions in horizon
        A_in_i = A_in;
        b_in_i = b_in;
        A_eq_i = A_eq;
        H_i = H_f;
        f_eps = [];
        mat_f_x0_i = mat_f_x0_free;
        if false && drop_comms
            f_tot = reshape(hor_ref(:,:,i,k-1), 1, [])*S_free*Phi;
        else
            f_tot = f_pf_free(:,:,i);
        end
    end
elseif comm_recovery == RecoveryType.perf
    A_in_i = A_in;
    b_in_i = b_in;
    A_eq_i = A_eq;
    H_i = H_f;
    f_eps = [];
    mat_f_x0_i = mat_f_x0_free;
    f_tot = f_pf_free(:,:,i);
elseif comm_recovery == RecoveryType.safety
    A_in_i = A_in;
    b_in_i = b_in;
    A_eq_i = A_eq;
    H_i = H_f;
    f_eps = [];
    mat_f_x0_i = mat_f_x0_free;
    f_tot = repmat(X0(1:3,i), k_hor, 1)'*S_free*Phi;
elseif comm_recovery == RecoveryType.bvc
    x_length = (d+1) * ndim * l;
    t_start = tic;

    [A_coll, b_coll] = BVC_constraints_ref(known_X0_ref, d, i, rmin, order, E1, E2, x_length, drop_comms);
    t_build(k,i) = toc(t_start);
    A_in_i = [A_in; A_coll];
    b_in_i = [b_in; b_coll];
    A_eq_i = A_eq;
    H_i = H_f;
    f_eps = [];
    mat_f_x0_i = mat_f_x0_free;
    f_tot = f_pf_free(:,:,i);
else
    A_in_i = A_in;
    b_in_i = b_in;
    A_eq_i = A_eq;
    H_i = H_f;
    f_eps = [];
    mat_f_x0_i = mat_f_x0_free;
    f_tot = 0.5*f_pf_free(:,:,i) + 0.5*reshape(hor_ref(:,:,i,k-1), 1, [])*S_free*Phi;
    for drone=1:N
        if drone ~= i
            distances = sqrt(sum(abs(hor_ref(:,:,i,k-1) - hor_ref(:,:,drone,k-1)).^2,1));
            scaling = (0.5*rmin_a ./ (distances)).^5 .* (linspace(1, 0, k_hor).^2);
            final_weights = interp1(linspace(1, (d+1)*l, k_hor), scaling, 1:(d+1)*l);
            repel_force = reshape(hor_ref(:,:,drone,k-1), 1, [])*S_free*Phi;
            for idx=0:(d+1)*l-1
                repel_force(idx*ndim+1:idx*ndim+3) = final_weights(idx+1) * repel_force(idx*ndim+1:idx*ndim+3);
            end
            f_tot = f_tot - 0.5*repel_force;
        end
    end
end
