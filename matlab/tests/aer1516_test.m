%%
% Load simulation parameters
load('sim_params.mat')

% Load MPC tuning parameters
load('mpc_params.mat')

% Choose what data to visualize
global debug_constr;
debug_constr = 0;

fail_type = {'sys', 'stoc'};
fprintf("STARTING EXPERIMENT - %s, %s, c=%f\n", comm_recovery, fail_type{comm_drop_stochastic+1}, c_comm)

k_hor = 10;
N = 15; % Num drones
num_trials = 5;
num_inner_trials = 5;
run_bvc = false;
run_alt = false;
use_repel = false;
deg_poly = 2;

comm_on_window = round(k_hor/c_comm) - k_hor;

plot = false;

for i = 1:N
    order(i) = order_a;
    rmin(i) = rmin_a;
    c(i,:) = c_a;
    E1(:,:,i) = E1_a;
    E2(:,:,i) = E2_a;
end

spd_r = min(k_hor, 10);
build_all_matrices;

pmin_gen = [-1.0, -1.0, 0.5]; %Area from which to sample init and goal states
pmax_gen = [1.0, 1.0, 2.0];

% Set algo params
if run_bvc
    use_ondemand = false;
    use_stateCA = false;
    use_softBVC = ~run_alt;
else
    use_ondemand = true;
    use_stateCA = run_alt;
    use_softBVC = false;
end

all_violations = [];
all_lengths = [];

tStart = tic;
for trial=1:num_trials
    % Generate a random set of initial positions
    po = gen_points(N, pmin_gen, pmax_gen, rmin, E1, order);

    % Do run
    aer1516_run;
    all_violations = [all_violations, violations];
    all_lengths = [all_lengths; lengths];
    fprintf("Trial %i - Violation: %i, Length: %f\n", trial, violations, sum(lengths)/num_inner_trials)
end
tEnd = toc(tStart);


fprintf("\n FINAL DATA - Violations: %f, Length: %f\n", sum(all_violations)/(num_trials*num_inner_trials), sum(sum(all_lengths))/(num_trials*num_inner_trials))

fname = sprintf("results/data_%s_%s_c0%i.mat", comm_recovery, fail_type{comm_drop_stochastic+1}, round(c_comm*10));
save(fname, "all_violations", "all_lengths", "tEnd")


if plot
    aer1516_plot
end
