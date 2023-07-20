%%%% No Control Variables

% Definition Variables


%%%%%%%% Freeway link %%%%%%%%
% traffic density class 1
%+1 considers the virtual density in section i+1 in the speed equation
rho_m1 = zeros(N_m(1,1)+1,K+1);
rho_m1(1:N_m(1,1),1) = rho_m1_initial(:,1);

rho_m2 = zeros(N_m(2,1)+1,K+1);
rho_m2(1:N_m(2,1),1) = rho_m2_initial(:,1);

rho_m3 = zeros(N_m(3,1)+1,K+1);
rho_m3(1:N_m(3,1),1) = rho_m3_initial(:,1);

rho_m4 = zeros(N_m(4,1)+1,K+1);
rho_m4(1:N_m(4,1),1) = rho_m4_initial(:,1);

rho_m5 = zeros(N_m(5,1)+1,K+1);
rho_m5(1:N_m(5,1),1) = rho_m5_initial(:,1);

rho_m6 = zeros(N_m(6,1)+1,K+1);
rho_m6(1:N_m(6,1),1) = rho_m6_initial(:,1);

rho_m7 = zeros(N_m(7,1)+1,K+1);
rho_m7(1:N_m(7,1),1) = rho_m7_initial(:,1);

rho_m8 = zeros(N_m(8,1)+1,K+1);
rho_m8(1:N_m(8,1),1) = rho_m8_initial(:,1);

rho_m9 = zeros(N_m(9,1)+1,K+1);
rho_m9(1:N_m(9,1),1) = rho_m9_initial(:,1);

rho_m10 = zeros(N_m(10,1)+1,K+1);
rho_m10(1:N_m(10,1),1) = rho_m10_initial(:,1);

rho_m11 = zeros(N_m(11,1)+1,K+1);
rho_m11(1:N_m(11,1),1) = rho_m11_initial(:,1);


% partial traffic density
partial_rho_m1 = zeros(J_m(1,1),N_m(1,1),K+1); % link 1
partial_rho_m2 = zeros(J_m(2,1),N_m(2,1),K+1); % link 2
partial_rho_m3 = zeros(J_m(3,1),N_m(3,1),K+1); % link 3
partial_rho_m4 = zeros(J_m(4,1),N_m(4,1),K+1); % link 4
partial_rho_m5 = zeros(J_m(5,1),N_m(5,1),K+1); % link 5
partial_rho_m6 = zeros(J_m(6,1),N_m(6,1),K+1); % link 6
partial_rho_m7 = zeros(J_m(7,1),N_m(7,1),K+1); % link 7
partial_rho_m8 = zeros(J_m(8,1),N_m(8,1),K+1); % link 8
partial_rho_m9 = zeros(J_m(9,1),N_m(9,1),K+1); % link 9
partial_rho_m10 = zeros(J_m(10,1),N_m(10,1),K+1); % link 10
partial_rho_m11 = zeros(J_m(11,1),N_m(11,1),K+1); % link 11


% 
% % composition rate  (portion of traffic volume with destination in J_m)
% %+1 considers the entering flow from the node to the first link

gamma_m1 = zeros(J_m(1,1),N_m(1,1)+1,K+1); % 
gamma_m1(1,:,1) = theta_o1(1,k); %destination D1
gamma_m1(2,:,1) = theta_o1(2,k); %destination D2

gamma_m2 = zeros(J_m(2,1),N_m(2,1)+1,K+1); % class 1 link 1
gamma_m2(1,:,1) = theta_o1(1,k); %destination D1
gamma_m2(2,:,1) = theta_o1(2,k); %destination D2

gamma_m3 = zeros(J_m(3,1),N_m(3,1)+1,K+1); 
gamma_m3(1,:,1) = 1; %destination D1
% gamma_m3(2,:,1) = 0.5; %destination D2

gamma_m4 = zeros(J_m(4,1),N_m(4,1)+1,K+1); 
gamma_m4(1,:,1) = 1; %destination D1
% gamma_m4(2,:,1) = 0.5; %destination D2


gamma_m5= zeros(J_m(5,1),N_m(5,1)+1,K+1); 
gamma_m5(1,:,1) = 1; %destination D1
% gamma_m5(2,:,1) = 0.5; %destination D2

gamma_m6 = zeros(J_m(6,1),N_m(6,1)+1,K+1); 
gamma_m6(1,:,1) = 0.5; %destination D1
gamma_m6(2,:,1) = 0.5; %destination D2

gamma_m7 = zeros(J_m(7,1),N_m(7,1)+1,K+1); 
gamma_m7(1,:,1) = 0.2; %destination D1
gamma_m7(2,:,1) = 0.8; %destination D2

gamma_m8 = zeros(J_m(8,1),N_m(8,1)+1,K+1); 
gamma_m8(1,:,1) = 1; %destination D1
% gamma_1_m8_nc(2,:,1) = 0.5; %destination D2

gamma_m9 = zeros(J_m(9,1),N_m(9,1)+1,K+1); 
gamma_m9(1,:,1) = 1; %destination D2

gamma_m10 = zeros(J_m(10,1),N_m(10,1)+1,K+1); 
gamma_m10(1,:,1) = 1; %destination D1

gamma_m11 = zeros(J_m(11,1),N_m(11,1)+1,K+1);
gamma_m11(1,:,1) = 1; %destination D2





for j=1:J_m(1,1)
    for i=1:N_m(1,1)
        partial_rho_m1(j,i,1)= rho_m1(i,1) * gamma_m1(j,i+1,1);
    end
end


for j=1:J_m(2,1)
    for i=1:N_m(2,1)
        partial_rho_m2(j,i,1)= rho_m2(i,1) * gamma_m2(j,i+1,1);
    end
end

for j=1:J_m(3,1)
    for i=1:N_m(3,1)
        partial_rho_m3(j,i,1)= rho_m3(i,1) * gamma_m3(j,i+1,1);
    end
end

for j=1:J_m(4,1)
    for i=1:N_m(4,1)
        partial_rho_m4(j,i,1)= rho_m4(i,1) * gamma_m4(j,i+1,1);
    end
end

for j=1:J_m(5,1)
    for i=1:N_m(5,1)
        partial_rho_m5(j,i,1)= rho_m5(i,1) * gamma_m5(j,i+1,1);
    end
end

for j=1:J_m(6,1)
    for i=1:N_m(6,1)
        partial_rho_m6(j,i,1)= rho_m6(i,1) * gamma_m6(j,i+1,1);
    end
end

for j=1:J_m(7,1)
    for i=1:N_m(7,1)
        partial_rho_m7(j,i,1)= rho_m7(i,1) * gamma_m7(j,i+1,1);
    end
end

for j=1:J_m(8,1)
    for i=1:N_m(8,1)
        partial_rho_m8(j,i,1)= rho_m8(i,1) * gamma_m8(j,i+1,1);
    end
end

for j=1:J_m(9,1)
    for i=1:N_m(9,1)
        partial_rho_m9(j,i,1)= rho_m9(i,1) * gamma_m9(j,i+1,1);
    end
end

for j=1:J_m(10,1)
    for i=1:N_m(10,1)
        partial_rho_m10(j,i,1)= rho_m10(i,1) * gamma_m10(j,i+1,1);
    end
end

for j=1:J_m(11,1)
    for i=1:N_m(11,1)
        partial_rho_m11(j,i,1)= rho_m11(i,1) * gamma_m11(j,i+1,1);
    end
end



% traffic flow 
%+1 considers the entering flow from the node to the first link
q_m1 = zeros(N_m(1,1)+1,K+1); % class 1 link 1
q_m1(2:N_m(1,1)+1,1) = q_m1_initial(:,1);

q_m2 = zeros(N_m(2,1)+1,K+1); %  link 2
q_m2(2:N_m(2,1)+1,1) = q_m2_initial(:,1);

q_m3 = zeros(N_m(3,1)+1,K+1); % link 3
q_m3(2:N_m(3,1)+1,1) = q_m3_initial(:,1);

q_m4 = zeros(N_m(4,1)+1,K+1); %link 4
q_m4(2:N_m(4,1)+1,1) = q_m4_initial(:,1);

q_m5 = zeros(N_m(5,1)+1,K+1); % link 5
q_m5(2:N_m(5,1)+1,1) = q_m5_initial(:,1);

q_m6= zeros(N_m(6,1)+1,K+1); %link 6
q_m6(2:N_m(6,1)+1,1) = q_m6_initial(:,1);

q_m7 = zeros(N_m(7,1)+1,K+1); % link 7
q_m7(2:N_m(7,1)+1,1) = q_m7_initial(:,1);

q_m8 = zeros(N_m(8,1)+1,K+1); %  link 8
q_m8(2:N_m(8,1)+1,1) = q_m8_initial(:,1);

q_m9 = zeros(N_m(9,1)+1,K+1); % link 9
q_m9(2:N_m(9,1)+1,1) = q_m9_initial(:,1);

q_m10 = zeros(N_m(10,1)+1,K+1); % link 10
q_m10(2:N_m(10,1)+1,1) = q_m10_initial(:,1);

q_m11 = zeros(N_m(11,1)+1,K+1); % link 11
q_m11(2:N_m(11,1)+1,1) = q_m11_initial(:,1);


q_d1 = zeros(1,K+1); % destination link 1
q_d2 = zeros(1,K+1); % destination link 2


% mean speed class 1
%+1 considers the mean speed of the entering flow from the node to the first link
v_m1 = zeros(N_m(1,1)+1,K+1); % link 1
v_m1(2:N_m(1,1)+1,1) = v_m1_initial(:,1);

v_m2 = zeros(N_m(2,1)+1,K+1); % link 2
v_m2(2:N_m(2,1)+1,1) = v_m2_initial(:,1);

v_m3 = zeros(N_m(3,1)+1,K+1); % link 3
v_m3(2:N_m(3,1)+1,1) = v_m3_initial(:,1);

v_m4 = zeros(N_m(4,1)+1,K+1); % link 4
v_m4(2:N_m(4,1)+1,1) = v_m4_initial(:,1);

v_m5 = zeros(N_m(5,1)+1,K+1); % link 5
v_m5(2:N_m(5,1)+1,1) = v_m5_initial(:,1);

v_m6 = zeros(N_m(6,1)+1,K+1); %link 6
v_m6(2:N_m(6,1)+1,1) = v_m6_initial(:,1);

v_m7 = zeros(N_m(7,1)+1,K+1); %link 7
v_m7(2:N_m(7,1)+1,1) = v_m7_initial(:,1);

v_m8 = zeros(N_m(8,1)+1,K+1); % link 8
v_m8(2:N_m(8,1)+1,1) = v_m8_initial(:,1);

v_m9 = zeros(N_m(9,1)+1,K+1); % link 9
v_m9(2:N_m(9,1)+1,1) = v_m9_initial(:,1);

v_m10 = zeros(N_m(10,1)+1,K+1); % link 10
v_m10(2:N_m(10,1)+1,1) = v_m10_initial(:,1);

v_m11 = zeros(N_m(11,1)+1,K+1); % link 11
v_m11(2:N_m(11,1)+1,1) = v_m11_initial(:,1);


%%%%%%%% Origin link %%%%%%%%

% partial origin demand o1
partial_demand_o1 = zeros(J_o(1,1),(K+1)); 

% origin flow class 1
q_o1 = zeros(1,(K+1)); % origin 1
% origin queue lenght class 1
l_o1 = zeros(1,(K+1)); %  origin 1

% partial origin queue lenght class 1
partial_l_o1 = zeros(J_o(1,1),(K+1)); %  origin 1

% composition rate (portion of traffic volume with destination in J_o)

gamma_o1 = zeros(J_o(1,1),K+1); % origin 1
gamma_o1(1,1) = theta_o1(1,1); %destination D1
gamma_o1(2,1) = theta_o1(2,1); %destination D2

%%%%%%%% On-ramp link %%%%%%%%

% partial on-ramp demand o3
partial_demand_o3 = zeros(J_r(1,1),(K+1)); % class 1 on-ramp o3

% on-ramp flow o3
q_o3 = zeros(1,(K+1));

% on-ramp queue lenght o3
l_o3 = zeros(1,(K+1)); 

% partial on-ramp queue lenght o3
partial_l_o3 = zeros(J_r(1,1),(K+1)); 

% composition rate (portion of traffic volume with destination in J_r)

gamma_o3 = zeros(J_r(1,1),K+1); % 
gamma_o3(1,1) = theta_o3(1,1); %destination D1

% partial on-ramp demand o5
partial_demand_o5 = zeros(J_r(2,1),(K+1)); 

% on-ramp flow o5
q_o5 = zeros(1,(K+1)); 

% on-ramp queue lenght o5
l_o5 = zeros(1,(K+1)); 

% partial on-ramp queue lenght o5
partial_l_o5 = zeros(J_r(2,1),(K+1)); 

% composition rate (portion of traffic volume with destination in J_r)

gamma_o5 = zeros(J_r(2,1),K+1); % 
gamma_o5(1,1) = theta_o5(1,1); %destination D1
gamma_o5(2,1) = theta_o5(2,1); %destination D2

% partial on-ramp demand o2
partial_demand_o2 = zeros(J_r(3,1),(K+1)); 

% on-ramp flow o2
q_o2 = zeros(1,(K+1)); 

% on-ramp queue lenght o2
l_o2 = zeros(1,(K+1)); 

% partial on-ramp queue lenght o2
partial_l_o2 = zeros(J_r(3,1),(K+1));

% composition rate (portion of traffic volume with destination in J_r)

gamma_o2 = zeros(J_r(3,1),K+1); 
gamma_o2(1,1) = theta_o2(1,1); %destination D1
gamma_o2(2,1) = theta_o2(2,1); %destination D2

% partial on-ramp demand o4
partial_demand_o4 = zeros(J_r(4,1),(K+1)); 

% on-ramp flow o4
q_o4 = zeros(1,(K+1)); 

% on-ramp queue lenght co4
l_o4 = zeros(1,(K+1)); 

% partial on-ramp queue lenght o4
partial_l_o4 = zeros(J_r(4,1),(K+1));


% composition rate (portion of traffic volume with destination in J_r)

gamma_o4 = zeros(J_r(4,1),K+1);
gamma_o4(1,1) = theta_o4(1,1); %destination D1


% partial on-ramp demand o6
partial_demand_o6 = zeros(J_r(5,1),(K+1)); %

% on-ramp flow o6
q_o6 = zeros(1,(K+1)); 

% on-ramp queue lenght o5
l_o6 = zeros(1,(K+1)); 


% partial on-ramp queue lenght o5
partial_l_o6 = zeros(J_r(5,1),(K+1)); 

% composition rate (portion of traffic volume with destination in J_r)

gamma_o6 = zeros(J_r(5,1),K+1); 
gamma_o6(1,1) = theta_o6(1,1); %destination D1


%%%%%%%% Node model %%%%%%%%

% incoming traffic flow

Q_n1 = zeros(J_n(1,1),(K+1)); % node 1
Q_n2 = zeros(J_n(2,1),(K+1)); % node 2
Q_n3 = zeros(J_n(3,1),(K+1)); % node 3
Q_n4 = zeros(J_n(4,1),(K+1)); % node 4
Q_n5 = zeros(J_n(5,1),(K+1)); % node 5
Q_n6 = zeros(J_n(6,1),(K+1)); % node 6
Q_n7 = zeros(J_n(7,1),(K+1)); % node 7
Q_n8 = zeros(J_n(8,1),(K+1)); % node 8
Q_n9 = zeros(J_n(9,1),(K+1)); % node 9

% splitting rate

beta_m1 = zeros (J_n(1,1),K+1);
beta_m1(1,:)=1; % rate of vehicles that will use the link M1 to get at destination 1 class 1%%
beta_m1(2,:)=1; % rate of vehicles that will use the link M1 to get at destination 2 class 1

beta_m2 = zeros (J_n(2,1),K+1);

beta_m2(1,:)=1; % rate of vehicles that use the link M2 to get at destination 1 class 1
beta_m2(2,:)=1; % rate of vehicles that use the link M2 to get at destination 2 class 1

beta_m3 = zeros (J_n(3,1),K+1);
beta_m6 = zeros (J_n(3,1),K+1);
beta_m3(1,:)=1; % rate of vehicles that will use the link M3 to get at destination 1 class 1
beta_m6(1,:)=0; % rate of vehicles that will use the link M6 to get at destination 1 class 1
beta_m6(2,:)=1;%0.45; % rate of vehicles that will use the link M6 to get at destination 2 class 1

beta_m4 = zeros (J_n(4,1),K+1);
beta_m4(1,:)=1; % rate of vehicles that will use the link M4 to get at destination 1 class 1
beta_m5 = zeros (J_n(5,1),K+1);
beta_m5(1,:)=1; % rate of vehicles that will use the link M5 to get at destination 1 class 1
beta_m10 = zeros (J_n(6,1),K+1);
beta_m10(1,:)=1; % rate of vehicles that will use the link M10 to get at destination 1 class 1
beta_m11 = zeros (J_n(9,1),K+1);
beta_m11(1,:)=1; % rate of vehicles that will use the link M11 to get at destination 2 class 1

beta_m7 = zeros (J_n(7,1),K+1);

beta_m7(1,:)=1; % rate of vehicles that will use the link M7 to get at destination 1 class 1
beta_m7(2,:)=1; % rate of vehicles that will use the link M7 to get at destination 2 class 1

beta_m8 = zeros (J_n(8,1),K+1);

beta_m8(1,:)=1; % rate of vehicles that will use the link M8 to get at destination 1 class 1
beta_m8(2,:)=0; % rate of vehicles that will use the link M8 to get at destination 2 class 1

beta_m9 = zeros (J_n(8,1),K+1);

beta_m9(1,:)=0; % rate of vehicles that will use the link M9 to get at destination 1 class 1
beta_m9(2,:)=1; % rate of vehicles that will use the link M9 to get at destination 2 class 1



% % accelerazioni
%
% a_seg_1_m1_nc=zeros(N_m(1,1),K+1);   % acc segmental
% a_seg_1_m2_nc=zeros(N_m(2,1),K+1);
% a_seg_1_m3_nc=zeros(N_m(3,1),K+1);
% a_seg_1_m4_nc=zeros(N_m(4,1),K+1);
% % a_seg_1_m5_nc=zeros(N_m(5,1),K+1);
% a_seg_1_m6_nc=zeros(N_m(6,1),K+1);
% a_seg_1_m7_nc=zeros(N_m(7,1),K+1);
% a_seg_1_m8_nc=zeros(N_m(8,1),K+1);
% a_seg_1_m9_nc=zeros(N_m(9,1),K+1);
% a_seg_1_m10_nc=zeros(N_m(10,1),K+1);
% a_seg_1_m11_nc=zeros(N_m(11,1),K+1);
% a_seg_1_m12_nc=zeros(N_m(12,1),K+1);
%
%
% a_cross_1_m1_nc=zeros(N_m(1,1),K+1);   % acc segmental
% a_cross_1_m2_nc=zeros(N_m(2,1),K+1);
% a_cross_1_m3_nc=zeros(N_m(3,1),K+1);
% a_cross_1_m4_nc=zeros(N_m(4,1),K+1);
% % a_cross_1_m5_nc=zeros(N_m(5,1),K+1);
% a_cross_1_m6_nc=zeros(N_m(6,1),K+1);
% a_cross_1_m7_nc=zeros(N_m(7,1),K+1);
% a_cross_1_m8_nc=zeros(N_m(8,1),K+1);
% a_cross_1_m9_nc=zeros(N_m(9,1),K+1);
% a_cross_1_m10_nc=zeros(N_m(10,1),K+1);
% a_cross_1_m11_nc=zeros(N_m(11,1),K+1);
% a_cross_1_m12_nc=zeros(N_m(12,1),K+1);
%
%
% a_arr_1_o3_nc=zeros(1,K+1);   % acc arrival vehicles
% a_w_1_o3_nc=zeros(1,K+1);     % acc waiting vehicles
% a_ls_1_o3_nc=zeros(1,K+1);    % acc leaving with stops vehicles
% a_lns_1_o3_nc=zeros(1,K+1);   % acc leaving without stops vehicles
% a_arr_1_o5_nc=zeros(1,K+1);   % acc arrival vehicles
% a_w_1_o5_nc=zeros(1,K+1);     % acc waiting vehicles
% a_ls_1_o5_nc=zeros(1,K+1);    % acc leaving with stops vehicles
% a_lns_1_o5_nc=zeros(1,K+1);   % acc leaving without stops vehicles
%
% a_arr_1_o2_nc=zeros(1,K+1);   % acc arrival vehicles
% a_w_1_o2_nc=zeros(1,K+1);     % acc waiting vehicles
% a_ls_1_o2_nc=zeros(1,K+1);    % acc leaving with stops vehicles
% a_lns_1_o2_nc=zeros(1,K+1);   % acc leaving without stops vehicles
% a_arr_1_o4_nc=zeros(1,K+1);   % acc arrival vehicles
% a_w_1_o4_nc=zeros(1,K+1);     % acc waiting vehicles
%
% a_ls_1_o4_nc=zeros(1,K+1);    % acc leaving with stops vehicles
% a_lns_1_o4_nc=zeros(1,K+1);   % acc leaving without stops vehicles
% a_arr_1_o5_nc=zeros(1,K+1);   % acc arrival vehicles
% a_w_1_o5_nc=zeros(1,K+1);     % acc waiting vehicles
% a_ls_1_o5_nc=zeros(1,K+1);    % acc leaving with stops vehicles
% a_lns_1_o5_nc=zeros(1,K+1);   % acc leaving without stops vehicles
%
% % numero di veicoli
%
% n_seg_1_m1_nc=zeros(N_m(1,1),K+1);   % n segmental
% n_seg_1_m2_nc=zeros(N_m(2,1),K+1);
% n_seg_1_m3_nc=zeros(N_m(3,1),K+1);
% n_seg_1_m4_nc=zeros(N_m(4,1),K+1);
% % n_seg_1_m5_nc=zeros(N_m(5,1),K+1);
% n_seg_1_m6_nc=zeros(N_m(6,1),K+1);
% n_seg_1_m7_nc=zeros(N_m(7,1),K+1);
% n_seg_1_m8_nc=zeros(N_m(8,1),K+1);
% n_seg_1_m9_nc=zeros(N_m(9,1),K+1);
% n_seg_1_m10_nc=zeros(N_m(10,1),K+1);
% n_seg_1_m11_nc=zeros(N_m(11,1),K+1);
% n_seg_1_m12_nc=zeros(N_m(12,1),K+1);
%
%
% n_cross_1_m1_nc=zeros(N_m(1,1),K+1); % n cross segmental
% n_cross_1_m2_nc=zeros(N_m(2,1),K+1);
% n_cross_1_m3_nc=zeros(N_m(3,1),K+1);
% n_cross_1_m4_nc=zeros(N_m(4,1),K+1);
% % n_cross_1_m5_nc=zeros(N_m(5,1),K+1);
% n_cross_1_m6_nc=zeros(N_m(6,1),K+1);
% n_cross_1_m7_nc=zeros(N_m(7,1),K+1);
% n_cross_1_m8_nc=zeros(N_m(8,1),K+1);
% n_cross_1_m9_nc=zeros(N_m(9,1),K+1);
% n_cross_1_m10_nc=zeros(N_m(10,1),K+1);
% n_cross_1_m11_nc=zeros(N_m(11,1),K+1);
% n_cross_1_m12_nc=zeros(N_m(12,1),K+1);
%
%
%  a_cross_1_m1_m2_nc = zeros(1,K+1);
%  a_cross_1_m1_m6_nc = zeros(1,K+1);
%
%  a_cross_1_m7_m8_nc = zeros(1,K+1);
%  a_cross_1_m7_m9_nc = zeros(1,K+1);
%
%  a_cross_1_m4_m10_nc = zeros(1,K+1);
%
%  n_cross_1_m1_m2_nc = zeros(1,K+1);
%  n_cross_1_m1_m6_nc = zeros(1,K+1);
%
%  n_cross_1_m7_m8_nc = zeros(1,K+1);
%  n_cross_1_m7_m9_nc = zeros(1,K+1);
%
%  n_cross_1_m4_m10_nc = zeros(1,K+1);
%
% n_arr_1_o3_nc=zeros(1,K+1);   % n arrival vehicles
% n_w_1_o3_nc=zeros(1,K+1);    % n waiting vehicles
% n_ls_1_o3_nc=zeros(1,K+1);   % n leaving with stops vehicles
% n_lns_1_o3_nc=zeros(1,K+1);   % n leaving without stops vehicles
% n_arr_1_o5_nc=zeros(1,K+1);   % n arrival vehicles
% n_w_1_o5_nc=zeros(1,K+1);    % n waiting vehicles
% n_ls_1_o5_nc=zeros(1,K+1);   % n leaving with stops vehicles
% n_lns_1_o5_nc=zeros(1,K+1);   % n leaving without stops vehicles
% n_arr_1_o2_nc=zeros(1,K+1);   % n arrival vehicles
% n_w_1_o2_nc=zeros(1,K+1);    % n waiting vehicles
% n_ls_1_o2_nc=zeros(1,K+1);   % n leaving with stops vehicles
% n_lns_1_o2_nc=zeros(1,K+1);   % n leaving without stops vehicles
% n_arr_1_o4_nc=zeros(1,K+1);   % n arrival vehicles
% n_w_1_o4_nc=zeros(1,K+1);    % n waiting vehicles
% n_ls_1_o4_nc=zeros(1,K+1);   % n leaving with stops vehicles
% n_lns_1_o4_nc=zeros(1,K+1);   % n leaving without stops vehicles