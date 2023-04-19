k = 2;
while ~reached_goal(pos_k_i(:,:,:), pf, 0.15, N)
    if comm_drop_stochastic
        drop_comms = rand(1) < c_comm;
    else
        if comm_window <= 0
            if drop_comms && c_comm > 0
                drop_comms = false;
                comm_window = comm_on_window/2 + randi(comm_on_window, 1) - 1;
            else
                drop_comms = true;
                comm_window = k_hor/2 + randi(k_hor, 1) - 1;
            end
        end
        comm_window = comm_window - 1;
    end

    if drop_comms
        num_iters_without_comm = num_iters_without_comm + 1;
    else
        num_iters_without_comm = 0;
    end

    % Update states for the regular agents
    for i = 1:N
        setup_mpc;

        % Solve QP
        t_start = tic;
        [sol, exitflag] = softMPC_update(l, deg_poly, A_in_i, b_in_i, A_eq_i, H_i,...
                                      mat_f_x0_i, f_tot, f_eps, X0(:,i), X0_ref(:,:,i));

        t_qp(k,i) = toc(t_start);
        if  isempty(sol)
            x = prev_x{i};
%             assert(~isempty(x), 'ERROR: No solution found - exitflag =  %i\n',exitflag);
        else
            prev_x{i} = sol;
            x = sol;
        end

        % Extract the control points
        u = x(1:size(mat_f_x0_free, 2));

        % Apply input to model starting form our previous init condition
        pos_i = reshape(Phi*u + A0.pos*X0(:,i),3, []);
        vel_i = reshape(Phi_vel*u + A0.vel*X0(:,i),3, []);

        % Sample at a higher frequency the interval 0:Ts:h-Ts
        % This tells us what should be the value of our state after
        % sending the optimal commands if the model was perfect
        pos_i_sample = reshape(Phi_sample*u + A0_s.pos*X0(:,i),3, []);
        vel_i_sample = reshape(Phi_vel_sample*u + A0_s.vel*X0(:,i),3, []);

        % Sample the resulting reference Bezier curves at 1/h and 1/Ts
        % Get the next input to be applied 'X0_ref'
        cols = 2 + (k-2)*(h/Ts):1 + (k-1)*(h/Ts);
        for r = 1:d+1
            rth_ref(:,:,r) = reshape(Der_h{r}*u, 3, []);
            rth_ref_sample(:,:,r) = reshape(Der_ts{r}*u, 3, []);
            X0_ref(:,r,i) = rth_ref(:,2,r);
            ref(:,k,r,i) = rth_ref(:,2,r);
            ref_sample(:,cols,r,i) = rth_ref_sample(:,:,r);
        end

        if ~drop_comms
            known_X0_ref = X0_ref;
        end

        % Simulate sending trajectories every Ts and applying at each time
        % step noise to the measurements and propagating the state forward
        X0_ex(:,1) = X0(:,i);
        for k_ex = 2:length(t_sample_r) + 1
            X0_ex(:, k_ex -1) = X0_ex(:, k_ex -1) + rnd_noise(std_p,std_v);
            X0_ex(:,k_ex) = model_s.A*X0_ex(:, k_ex-1) + model_s.B*rth_ref_sample(:, k_ex-1, 1);
        end

        % Initial conditions for next MPC cycle - based on sensing
        X0(:,i) = X0_ex(:, end);

        % Update agent's states at 1/h and 1/Ts frequencies
        pos_k_i_sample(:,cols,i) = X0_ex(1:3, 2:end);
        vel_k_i_sample(:,cols,i) = X0_ex(4:6, 2:end);

        pred_X0(:,i) = [pos_i_sample(:,end); vel_i_sample(:,end)];
        pos_k_i(:,k,i) = X0(1:3,i);
        vel_k_i(:,k,i) = X0(4:6,i);

        % Reference and state prediction horizons - visualization purposes
        if ~drop_comms
            hor_ref(:,:,i,k) = rth_ref(:,:,1);
        else
            hor_ref(:,1:end-1,i,k) = hor_ref(:,1:end-1,i,k-1);
            hor_ref(:,k_hor,i,k) = hor_ref(:,end,i,k-1) + max(0, (10-num_iters_without_comm)/10)*(hor_ref(:,end,i,k-1) - hor_ref(:,end-1,i,k-1));
        end

        hor_rob_k(:,:,i) = [X0(1:3,i) pos_i(:,1:end)];
    end

    k = k+1;
end
