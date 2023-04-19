clc
clear all
close all
warning('off','all')

comm_types = enumeration('RecoveryType');

for comm_recovery_idx = 1:length(comm_types)
    comm_recovery = comm_types(comm_recovery_idx);
    for c_comm = 0.2:0.2:0.8
        for comm_drop_stochastic=0:1
            aer1516_test
            clearvars -except comm_types comm_recovery c_comm comm_drop_stochastic
        end
    end
end
