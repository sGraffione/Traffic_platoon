%%% The follower platoon starts its journey from section 1 of link m7
%%% Rendez-vous point with the leader platoon is at the beginning of section 1 of link m5

%% Data

%% Meeting point parameters

p_bar_follower2 = N_m(7,1)*Delta_M(7,1) + N_m(8,1)*Delta_M(8,1) + N_m(2,1)*Delta_M(2,1)+ N_m(6,1)*Delta_M(6,1) +15*Delta_M(6,1);  % position of the meeting point


% k_bar_follower2 = 130; % is the time step in which the meeting should occur

Delta_k_follower = 1; %

K_fin_follower2 = k_bar_follower2 + Delta_k_follower;

%% Platoon parameters

v_max_follower2 = 75; % maximum speed of platoon

v_min_follower2 = 50; % minimum speed of platoon

% p_iniz_follower2 = 0; %starting position of platoon

M_path_follower2 = 4; % number of links included in the path of platoon 

N_path_follower2 = N_m(7,1) +  N_m(8,1)+ N_m(2,1) +  N_m(6,1) + 15; % number of sections included in the path of platoon 

delta_v = 0.2;

%% Freeway parameters

v_traffic_follower2 = zeros(N_path_follower2,K_fin_follower2); %speed of traffic in platoon path

v_traffic_follower2(1:N_m(2,1),:) = v_m2(2:N_m(2,1)+1,x-j1+l+K_follower:x-j1+K_fin_follower2+K_follower-h);
v_traffic_follower2(N_m(2,1)+ 1 : N_m(2,1)+ N_m(6,1) +1 ,:) = v_m6(:,x-j1+l+K_follower:x-j1+K_fin_follower2+K_follower-h);
v_traffic_follower2(N_m(2,1)+ N_m(6,1)+1 : N_m(2,1)+ N_m(6,1)+ N_m(7,1) +1 , :) = v_m7(:,x-j1+l+K_follower:x-j1+K_fin_follower2-h+K_follower);
v_traffic_follower2(N_m(2,1)+ N_m(6,1)+ N_m(7,1) +1 : N_m(2,1)+ N_m(6,1)+ N_m(7,1)+ N_m(8,1) +1 , :) = v_m8(:,x-j1+l+K_follower:x-j1+K_fin_follower2-h+K_follower);
v_traffic_follower2(N_m(2,1)+ N_m(6,1)+ N_m(7,1)+ N_m(8,1) +2 : N_m(2,1)+ N_m(6,1)+ N_m(7,1)+ N_m(8,1) +4 , :) = v_m2(N_m(2,1)-1:N_m(2,1)+1,x-j1+l+K_follower:x-j1+K_fin_follower2-h+K_follower);
v_traffic_follower2(N_m(2,1)+ N_m(6,1)+ N_m(7,1)+ N_m(8,1) +5 : N_m(2,1)+ N_m(6,1)+ N_m(7,1)+ N_m(8,1) +7 , :) = v_m6(N_m(6,1)-2:N_m(6,1),x-j1+l+K_follower:x-j1+K_fin_follower2-h+K_follower);
v_traffic_follower2(N_m(2,1)+ N_m(6,1)+ N_m(7,1)+ N_m(8,1) +8 : N_m(2,1)+ N_m(6,1)+ N_m(7,1)+ N_m(8,1) +10 , :) = v_m11(N_m(11,1)-6:N_m(11,1)-4,x-j1+l+K_follower:x-j1+K_fin_follower2-h+K_follower);
v_traffic_follower2(N_m(2,1)+ N_m(6,1)+ N_m(7,1)+ N_m(8,1) +11 : N_m(2,1)+ N_m(6,1)+ N_m(7,1)+ N_m(8,1) +13 , :) = v_m11(N_m(11,1)-3:N_m(11,1)-1,x-j1+l+K_follower:x-j1+K_fin_follower2-h+K_follower);
v_traffic_follower2(N_m(2,1)+ N_m(6,1)+ N_m(7,1)+ N_m(8,1) +14 : N_m(2,1)+ N_m(6,1)+ N_m(7,1)+ N_m(8,1) +16 , :) = v_m8(N_m(8,1)-1:N_m(8,1)+1,x-j1+l+K_follower:x-j1+K_fin_follower2-h+K_follower);
% v_traffic_follower2(N_m(2,1)+ N_m(6,1)+ N_m(7,1)+ N_m(8,1) +17 : N_m(2,1)+ N_m(6,1)+ N_m(7,1)+ N_m(8,1) +19 , :) = v_m8(N_m(8,1)-1:N_m(8,1)+1,x-j1+2:B+1);



vMinFollower2 = zeros(N_path_follower2,K_fin_follower2); % lowest value between min speed and speed of traffic in each section at each time step
for k = 1:K_fin_follower2

    for i = 1: N_path_follower2
        if (v_min_follower2 < v_traffic_follower2(i,k))
        
            vMinFollower2(i,k) = v_min_follower2;
        else
            vMinFollower2(i,k) = v_traffic_follower2(i,k);
        end
    end
end

posiz_sez_follower2 = zeros (1,N_path_follower2); %initial position of each section belonging to the platoon path

for i = 2 : N_path_follower2

    if i >= 2 && i<= N_m(2,1) + 1

        posiz_sez_follower2(1,i) = (i-1) * Delta_M(2,1);

    elseif i > N_m(2,1) + 1 && i <= N_m(2,1) + N_m(6,1) +1

        posiz_sez_follower2(1,i) = (i-1) * Delta_M(6,1);

    elseif i > N_m(2,1) + N_m(6,1) + 1 && i <= N_m(2,1) + N_m(6,1) + N_m(7,1) +1

        posiz_sez_follower2(1,i) = (i-1) * Delta_M(7,1);

    elseif i > N_m(2,1) + N_m(6,1)+ N_m(7,1) + 1 && i <= N_m(2,1) + N_m(6,1) + N_m(7,1) + N_m(8,1) +1

        posiz_sez_follower2(1,i) = (i-1) * Delta_M(8,1);

    else
        posiz_sez_follower2(1,i) = (i-1) * Delta_M(8,1);

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

p_follower2 = sdpvar(1,K_fin_follower2); % position of the platoon

%% Control variable

v_follower2 = sdpvar(1,K_fin_follower2); % speed of the platoon

%% Decision variable

z2 = binvar(1,1); % equal to 1 if the follower decides to merge with the leader, 0 otherwise

%% Ausiliary variables

y_follower2 = binvar(N_path_follower2,K_fin_follower2); % equal to one if the platoon is after the section i in k

w_follower2 = binvar(N_path_follower2,K_fin_follower2); % equal to one if the platoon is before the section i in k

lambda_follower2 = binvar(N_path_follower2,K_fin_follower2); % equal to one if the platoon is into section i in k



%% Objective function

Objective_follower2 = beta_1 * z2 - beta_2 * z2 * (p_follower2(1,k_bar_follower2) - p_bar_follower2);


%% constraints

%constraint (2) platoon position

ConstrPos12 = [];

for k = 1:K_fin_follower2-1
    ConstrPos12 = [ ConstrPos12, p_follower2(1,k+1) == p_follower2(1,k) + v_follower2(1,k)*(T)];
end

%constraint on the initial position of the platoon

ConstrPosIniz12 = [];

ConstrPosIniz12 =[ConstrPosIniz12, p_follower2(1,1) == p_iniz_follower2];

% %constraint on the initial speed of the platoon
% 
% ConstrSpeedIniz2 = [];
% 
% ConstrSpeedIniz2 =[ConstrSpeedIniz2, v_follower2(1,1) == v_iniz_follower2];


%constraint (3) y part 1

ConstrYp12 = [];

for k = 1:K_fin_follower2
    for i = 1: N_path_follower2
        ConstrYp12 = [ ConstrYp12, (p_follower2(1,k) - posiz_sez_follower2(1,i)) + BigM*( 1 - y_follower2(i,k) ) >= epsilon ];
    end
end


%constraint (4) y part 3

ConstrYp22 = [];

for k = 1:K_fin_follower2
    for i = 1: N_path_follower2
        ConstrYp22 = [ConstrYp22, (posiz_sez_follower2(1,i) - p_follower2(1,k)) + BigM * y_follower2(i,k) >= 0];
    end
end


%constraint (5) w part 1

ConstrWp12 = [];

for k = 1:K_fin_follower2
    for i = 1: N_path_follower2 - 1
        ConstrWp12 = [ConstrWp12, (p_follower2(1,k) - posiz_sez_follower2(1,i+1)) + BigM * w_follower2(i,k) >= epsilon ];
    end
end


%constraint (6) w part 2

ConstrWp22 = [];

for k = 1:K_fin_follower2
    for i = 1: N_path_follower2 - 1
        ConstrWp22 = [ConstrWp22, (posiz_sez_follower2(1,i+1) - p_follower2(1,k)) + BigM * (1- w_follower2(i,k)) >= 0];
    end
end


%constraint (7) lambda part 1

Constrlambdap12 = [];

for k = 1:K_fin_follower2
    for i = 1: N_path_follower2
        Constrlambdap12 =  [Constrlambdap12,lambda_follower2(i,k) <= y_follower2(i,k)] ;
    end
end



%constraint (8) lambda part 2

Constrlambdap22 = [];

for k = 1:K_fin_follower2
    for i = 1: N_path_follower2
        Constrlambdap22 = [Constrlambdap22,lambda_follower2(i,k) <= w_follower2(i,k)];
    end
end

%constraint (9) lambda part 3

Constrlambdap32 = [];

for k = 1:K_fin_follower2
    for i = 1: N_path_follower2
        Constrlambdap32 = [Constrlambdap32, lambda_follower2(i,k) >= y_follower2(i,k) + w_follower2(i,k) - 1];
    end
end


%constraint (10) traffic speed

ConstrSpeedTraf2 = [];


for k = 2:K_fin_follower2

    somma42 = 0;

    for i = 1: N_path_follower2
        somma42 = somma42 + lambda_follower2(i,k) * v_traffic_follower2(i,k);
    end

    ConstrSpeedTraf2 = [ConstrSpeedTraf2, v_follower2(1,k) <= somma42];
end

%constraint1 (11) minimum speed of platoon

ConstrSpeedPos2=[];

for k = 1:K_fin_follower2
    ConstrSpeedPos2 = [ConstrSpeedPos2, v_follower2(1,k) >= 0];
end


%constraint (12) maximum speed of platoon

ConstrSpeedMax2 = [];

for k = 1:K_fin_follower2
    ConstrSpeedMax2 = [ConstrSpeedMax2, v_follower2(1,k) <= v_max_follower2];
end


%constraint2 (13) minimum speed of platoon

ConstrSpeedMin2 = [];

for k = 1:K_fin_follower2

    somma32=0;

    for i = 1: N_path_follower2
        somma32 = somma32 + lambda_follower2(i,k)*vMinFollower2(i,k);
    end

    ConstrSpeedMin2 = [ConstrSpeedMin2, v_follower2(1,k) >= somma32];
end


%constraint (14) speed difference upper limit 

ConstrDeltaSpeedMax2 = [];

for k = 1:K_fin_follower2-1

   ConstrDeltaSpeedMax2 = [ConstrDeltaSpeedMax2, (v_follower2(1,k+1) - v_follower2(1,k)) <= delta_v];

end


%constraint (15) speed difference lower limit 

ConstrDeltaSpeedMin2 = [];

for k = 1:K_fin_follower2-1

   ConstrDeltaSpeedMin2 = [ConstrDeltaSpeedMin2, (v_follower2(1,k+1) - v_follower2(1,k)) >= -delta_v];

end

% constraint (16) speed difference upper limit for initial speed

ConstrDeltaSpeedInizMax2 = [];

   ConstrDeltaSpeedInizMax2 = [ConstrDeltaSpeedInizMax2, (v_follower2(1,1) - v_iniz_follower2) <= delta_v];



%constraint (17) speed difference lower limit for initial speed

ConstrDeltaSpeedInizMin2 = [];

   ConstrDeltaSpeedInizMin2 = [ConstrDeltaSpeedInizMin2, (v_follower2(1,1) - v_iniz_follower2) >= -delta_v];


%constraint (16) late or early

ConstrFinalPos2 = [];
ConstrFinalPos2 = [ConstrFinalPos2, p_follower2(1,k_bar_follower2) - p_bar_follower2 + sigma >= -p_bar_follower2 + (p_bar_follower2 + epsilon)*z2];


ConstraintsF2=[ConstrPos12, ConstrDeltaSpeedInizMax2, ConstrDeltaSpeedInizMin2, ConstrPosIniz12, ConstrYp12, ConstrYp22, ConstrWp12, ConstrWp22, Constrlambdap12,...
Constrlambdap22, Constrlambdap32, ConstrSpeedTraf2, ConstrSpeedPos2,ConstrSpeedMax2,ConstrSpeedMin2,ConstrFinalPos2,ConstrDeltaSpeedMin2,ConstrDeltaSpeedMax2];

% Risolvo con Gurobi

options=sdpsettings('solver','gurobi');
optimize(ConstraintsF2,-Objective_follower2,options);

% Risolvo con Yalmip

%optimize(Constraints,Objective);

% Scrivo la soluzioni in matrici matlab
%
solutionp_follower2=value(p_follower2);
solutionv_follower2=value(v_follower2);
solutiony_follower2=value(y_follower2);
solutionw_follower2=value(w_follower2);
solutionlambda_follower2=value(lambda_follower2);
solutionz2=value(z2);
optimum_follower2=value(Objective_follower2);
