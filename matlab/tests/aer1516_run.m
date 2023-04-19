%%%%%%%%%%%%%%%% RUN ALGORITHM %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

drop_comms = false;
num_iters_without_comm = 0;
comm_window = randi(k_hor, 1);
all_states = [];
goals = [];
lengths = [];

for inner_trial = 1:num_inner_trials
    %%%%%%%%%%%%%%%% INIT ALGORITHM %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    pf = gen_points(N, pmin_gen, pmax_gen, rmin, E1, order);
    init_mpc;

    %%%%%%%%%%%%%%% MPC MAIN LOOP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    num_repels = zeros(N,1);
    solve_mpc

    goals = cat(1, goals, pf);
    all_states = cat(2, all_states, pos_k_i);
    lengths = [lengths, k];
end

%% POST-CHECK OF SOLUTION

% Check if collision constraints were not violated
violations = 0;
rmin_check = 0.2;
c_check = 2.25;
E_check = diag([1,1,c_check]);
E1_check = E_check^(-1);

for i = 1:N
    for j = 1:N
        if(i~=j)
            differ = E1_check*(all_states(:,:,i) - all_states(:,:,j));
            dist = (sum(differ.^order(j),1)).^(1/order(j));
            if min(dist) < (rmin_check)
                violations = violations + 1;
            end
        end
    end
end

violations = violations/2;
