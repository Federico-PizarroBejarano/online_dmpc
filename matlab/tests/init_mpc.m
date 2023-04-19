for i=1:N
    f_pf_free(:,:,i) = repmat(pf(:,:,i)', k_hor, 1)'*S_free*Phi;
    f_pf_obs(:,:,i) = repmat(pf(:,:,i)', k_hor, 1)'*S_obs*Phi; 
    if inner_trial == 1
       poi = po(:,:,i)';
       voi = 0.001*ones(3, 1);
       hor_ref(:,:,i,1) = repmat(poi, 1, k_hor);
       hor_rob(:,:,i,1) = repmat(poi, 1, k_hor+1);
       X0(:,i) = [poi; voi];
       prev_state(:,i) = X0(:,i);
    else
       poi = pos_k_i(:, end, i);
       voi = vel_k_i(:, end, i);
       hor_ref(:,:,i,1) = hor_ref(:,:,i,end);
       hor_rob(:,:,i,1) = hor_rob(:,:,i,end);
    end
    pos_k_i(:,1,i) = poi;
    vel_k_i(:,1,i) = voi;
    pos_k_i_sample(:,1,i) = poi;
    X0_ref(:,:,i) = [poi, voi, zeros(3,d - 1)];
    known_X0_ref = X0_ref;
    for r = 1:deg_poly+1
        ref(:,1,r,i) = X0_ref(:,r,i); 
        ref_sample(:,1,r,i) = X0_ref(:,r,i);
   end
end

pred_X0 = X0;

% Variables for reference replanning based on state feedback
integ_err(:,1) = zeros(3, 1);