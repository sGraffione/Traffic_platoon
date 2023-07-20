%%% The follower platoon starts its journey from section 1 of link m7
%%% Rendez-vous point with the leader platoon is at the beginning of section 1 of link m5

%% Data

%% Meeting point parameters

p_bar_follower1 = N_m(7,1)*Delta_M(7,1) + N_m(8,1)*Delta_M(8,1) + N_m(2,1)*Delta_M(2,1)+ N_m(6,1)*Delta_M(6,1) +11*Delta_M(6,1);  % position of the meeting point


% k_bar_follower1 = 50; % is the time step in which the meeting should occur

Delta_k_follower = 1; %

K_fin_follower1 = k_bar_follower1 + Delta_k_follower;

%% Platoon parameters

v_max_follower1 = 75; % maximum speed of platoon

v_min_follower1 = 50; % minimum speed of platoon

% p_iniz_follower1 = 0; %starting position of platoon

M_path_follower1 = 4; % number of links included in the path of platoon 

N_path_follower1 = N_m(7,1) +  N_m(8,1)+ N_m(2,1) +  N_m(6,1) + 11; % number of sections included in the path of platoon 

delta_v = 0.2;

%% Freeway parameters

v_traffic_follower1 = zeros(N_path_follower1,K_fin_follower1); %speed of traffic in platoon path

v_traffic_follower1(1:N_m(2,1),:) = v_m2(2:N_m(2,1)+1,K_follower1+1+x:x+K_follower1+K_fin_follower1);
v_traffic_follower1(N_m(2,1)+ 1 : N_m(2,1)+ N_m(6,1) +1 ,:) = v_m6(:,K_follower1+1+x:x+K_follower1+K_fin_follower1);
v_traffic_follower1(N_m(2,1)+ N_m(6,1)+1 : N_m(2,1)+ N_m(6,1)+ N_m(7,1) +1 , :) = v_m7(:,K_follower1+1+x:x+K_follower1+K_fin_follower1);
v_traffic_follower1(N_m(2,1)+ N_m(6,1)+ N_m(7,1) +1 : N_m(2,1)+ N_m(6,1)+ N_m(7,1)+ N_m(8,1) +1 , :) = v_m8(:,K_follower1+1+x:x+K_follower1+K_fin_follower1);
v_traffic_follower1(N_m(2,1)+ N_m(6,1)+ N_m(7,1)+ N_m(8,1) +2 : N_m(2,1)+ N_m(6,1)+ N_m(7,1)+ N_m(8,1) +4 , :) = v_m11(N_m(11,1)-1:N_m(11,1)+1,K_follower1+1+x:x+K_follower1+K_fin_follower1);
v_traffic_follower1(N_m(2,1)+ N_m(6,1)+ N_m(7,1)+ N_m(8,1) +5 : N_m(2,1)+ N_m(6,1)+ N_m(7,1)+ N_m(8,1) +7 , :) = v_m6(N_m(11,1)-6:N_m(11,1)-4,K_follower1+1+x:x+K_follower1+K_fin_follower1);
v_traffic_follower1(N_m(2,1)+ N_m(6,1)+ N_m(7,1)+ N_m(8,1) +8 : N_m(2,1)+ N_m(6,1)+ N_m(7,1)+ N_m(8,1) +10 , :) = v_m7(N_m(7,1)-3:N_m(7,1)-1,K_follower1+1+x:x+K_follower1+K_fin_follower1);
v_traffic_follower1(N_m(2,1)+ N_m(6,1)+ N_m(7,1)+ N_m(8,1) +11 : N_m(2,1)+ N_m(6,1)+ N_m(7,1)+ N_m(8,1) +12 , :) = v_m8(N_m(8,1):N_m(8,1)+1,K_follower1+1+x:x+K_follower1+K_fin_follower1);



vMinFollower1 = zeros(N_path_follower1,K_fin_follower1); % lowest value between min speed and speed of traffic in each section at each time step
for k = 1:K_fin_follower1

    for i = 1: N_path_follower1
        if (v_min_follower1 < v_traffic_follower1(i,k))
        
            vMinFollower1(i,k) = v_min_follower1;
        else
            vMinFollower1(i,k) = v_traffic_follower1(i,k);
        end
    end
end

posiz_sez_follower1 = zeros (1,N_path_follower1); %initial position of each section belonging to the platoon path

for i = 2 : N_path_follower1

    if i >= 2 && i<= N_m(2,1) + 1

        posiz_sez_follower1(1,i) = (i-1) * Delta_M(2,1);

    elseif i > N_m(2,1) + 1 && i <= N_m(2,1) + N_m(6,1) +1

        posiz_sez_follower1(1,i) = (i-1) * Delta_M(6,1);

    elseif i > N_m(2,1) + N_m(6,1) + 1 && i <= N_m(2,1) + N_m(6,1) + N_m(7,1) +1

        posiz_sez_follower1(1,i) = (i-1) * Delta_M(7,1);

    elseif i > N_m(2,1) + N_m(6,1)+ N_m(7,1) + 1 && i <= N_m(2,1) + N_m(6,1) + N_m(7,1) + N_m(8,1) +1

        posiz_sez_follower1(1,i) = (i-1) * Delta_M(8,1);

    else
        posiz_sez_follower1(1,i) = (i-1) * Delta_M(8,1);

    end
end


%% Weights of the objective funtion

beta_1 = 10;

beta_2 = 1;

%% Other parameters

BigM = 100000;

epsilon = 0.001;

sigma = 0.002;

%% Platoon state variable

p_follower1 = sdpvar(1,K_fin_follower1); % position of the platoon

%% Control variable

v_follower1 = sdpvar(1,K_fin_follower1); % speed of the platoon

%% Decision variable

z1 = binvar(1,1); % equal to 1 if the follower decides to merge with the leader, 0 otherwise

%% Ausiliary variables

y_follower1 = binvar(N_path_follower1,K_fin_follower1); % equal to one if the platoon is after the section i in k

w_follower1 = binvar(N_path_follower1,K_fin_follower1); % equal to one if the platoon is before the section i in k

lambda_follower1 = binvar(N_path_follower1,K_fin_follower1); % equal to one if the platoon is into section i in k



%% Objective function

Objective_follower1 = beta_1 * z1 - beta_2 * z1 * (p_follower1(1,k_bar_follower1) - p_bar_follower1);


%% constraints

%constraint (2) platoon position

ConstrPos11 = [];

for k = 1:K_fin_follower1-1
    ConstrPos11 = [ ConstrPos11, p_follower1(1,k+1) == p_follower1(1,k) + v_follower1(1,k)*(T)];
end

%constraint on the initial position of the platoon

ConstrPosIniz11 = [];

ConstrPosIniz11 =[ConstrPosIniz11, p_follower1(1,1) == p_iniz_follower1];

% constraint on the initial speed of the platoon

% ConstrSpeedIniz1 = [];
% 
% ConstrSpeedIniz1 =[ConstrSpeedIniz1, v_follower1(1,1) == v_iniz_follower1];


%constraint (3) y part 1

ConstrYp11 = [];

for k = 1:K_fin_follower1
    for i = 1: N_path_follower1
        ConstrYp11 = [ ConstrYp11, (p_follower1(1,k) - posiz_sez_follower1(1,i)) + BigM*( 1 - y_follower1(i,k) ) >= epsilon ];
    end
end


%constraint (4) y part 3

ConstrYp21 = [];

for k = 1:K_fin_follower1
    for i = 1: N_path_follower1
        ConstrYp21 = [ConstrYp21, (posiz_sez_follower1(1,i) - p_follower1(1,k)) + BigM * y_follower1(i,k) >= 0];
    end
end


%constraint (5) w part 1

ConstrWp11 = [];

for k = 1:K_fin_follower1
    for i = 1: N_path_follower1 - 1
        ConstrWp11 = [ConstrWp11, (p_follower1(1,k) - posiz_sez_follower1(1,i+1)) + BigM * w_follower1(i,k) >= epsilon ];
    end
end


%constraint (6) w part 2

ConstrWp21 = [];

for k = 1:K_fin_follower1
    for i = 1: N_path_follower1 - 1
        ConstrWp21 = [ConstrWp21, (posiz_sez_follower1(1,i+1) - p_follower1(1,k)) + BigM * (1- w_follower1(i,k)) >= 0];
    end
end


%constraint (7) lambda part 1

Constrlambdap11 = [];

for k = 1:K_fin_follower1
    for i = 1: N_path_follower1
        Constrlambdap11 =  [Constrlambdap11,lambda_follower1(i,k) <= y_follower1(i,k)] ;
    end
end



%constraint (8) lambda part 2

Constrlambdap21 = [];

for k = 1:K_fin_follower1
    for i = 1: N_path_follower1
        Constrlambdap21 = [Constrlambdap21,lambda_follower1(i,k) <= w_follower1(i,k)];
    end
end

%constraint (9) lambda part 3

Constrlambdap31 = [];

for k = 1:K_fin_follower1
    for i = 1: N_path_follower1
        Constrlambdap31 = [Constrlambdap31, lambda_follower1(i,k) >= y_follower1(i,k) + w_follower1(i,k) - 1];
    end
end


%constraint (10) traffic speed

ConstrSpeedTraf1 = [];


for k = 2:K_fin_follower1

    somma41 = 0;

    for i = 1: N_path_follower1
        somma41 = somma41 + lambda_follower1(i,k) * v_traffic_follower1(i,k);
    end

    ConstrSpeedTraf1 = [ConstrSpeedTraf1, v_follower1(1,k) <= somma41];
end

%constraint1 (11) minimum speed of platoon

ConstrSpeedPos1=[];

for k = 1:K_fin_follower1
    ConstrSpeedPos1 = [ConstrSpeedPos1, v_follower1(1,k) >= 0];
end


%constraint (12) maximum speed of platoon

ConstrSpeedMax1 = [];

for k = 1:K_fin_follower1
    ConstrSpeedMax1 = [ConstrSpeedMax1, v_follower1(1,k) <= v_max_follower1];
end


%constraint2 (13) minimum speed of platoon

ConstrSpeedMin1 = [];

for k = 1:K_fin_follower1

    somma31=0;

    for i = 1: N_path_follower1
        somma31 = somma31 + lambda_follower1(i,k)*vMinFollower1(i,k);
    end

    ConstrSpeedMin1 = [ConstrSpeedMin1, v_follower1(1,k) >= somma31];
end


%constraint (14) speed difference upper limit 

ConstrDeltaSpeedMax1 = [];

for k = 1:K_fin_follower1-1

   ConstrDeltaSpeedMax1 = [ConstrDeltaSpeedMax1, (v_follower1(1,k+1) - v_follower1(1,k)) <= delta_v];

end


%constraint (15) speed difference lower limit 

ConstrDeltaSpeedMin1 = [];

for k = 1:K_fin_follower1-1

   ConstrDeltaSpeedMin1 = [ConstrDeltaSpeedMin1, (v_follower1(1,k+1) - v_follower1(1,k)) >= -delta_v];

end
% 
% constraint (16) speed difference upper limit for initial speed

ConstrDeltaSpeedInizMax1 = [];

   ConstrDeltaSpeedInizMax1 = [ConstrDeltaSpeedInizMax1, (v_follower1(1,1) - v_iniz_follower1) <= delta_v];



%constraint (17) speed difference lower limit for initial speed

ConstrDeltaSpeedInizMin1 = [];

   ConstrDeltaSpeedInizMin1 = [ConstrDeltaSpeedInizMin1, (v_follower1(1,1) - v_iniz_follower1) >= -delta_v];


%constraint (18) late or early

ConstrFinalPos1 = [];
ConstrFinalPos1 = [ConstrFinalPos1, p_follower1(1,k_bar_follower1) - p_bar_follower1 + sigma >= -p_bar_follower1 + (p_bar_follower1 + epsilon)*z1];


ConstraintsF1=[ConstrPos11, ConstrPosIniz11, ConstrDeltaSpeedInizMax1, ConstrDeltaSpeedInizMin1, ConstrYp11, ConstrYp21, ConstrWp11, ConstrWp21, Constrlambdap11,...
Constrlambdap21, Constrlambdap31, ConstrSpeedTraf1, ConstrSpeedPos1,ConstrSpeedMax1,ConstrSpeedMin1,ConstrFinalPos1,ConstrDeltaSpeedMin1,ConstrDeltaSpeedMax1];

% Risolvo con Gurobi

options=sdpsettings('solver','gurobi');
optimize(ConstraintsF1,-Objective_follower1,options);

% Risolvo con Yalmip

%optimize(Constraints,Objective);

% Scrivo la soluzioni in matrici matlab
%
solutionp_follower1=value(p_follower1);
solutionv_follower1=value(v_follower1);
solutiony_follower1=value(y_follower1);
solutionw_follower1=value(w_follower1);
solutionlambda_follower1=value(lambda_follower1);
solutionz1=value(z1);
optimum_follower1=value(Objective_follower1);
