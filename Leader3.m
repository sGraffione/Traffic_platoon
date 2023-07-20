
%%% The leader platoon starts its journey starting from section 1 of link m1
%%% its destination is at the end of section 4 of link m10

%% Data

%% Meeting point parameters

p_bar3 = N_m(1,1)*Delta_M(1,1) + N_m(2,1)*Delta_M(2,1) + N_m(3,1)*Delta_M(3,1) + N_m(4,1)*Delta_M(4,1) + N_m(5,1)*Delta_M(5,1) + N_m(10,1)*Delta_M(10,1) + 10*Delta_M(10,1); % position of the meeting point

% k_bar3 = 150; % is the time step in which the meeting should occur

Delta_k = 5; %

K_fin3 = k_bar3 + Delta_k;

%% Platoon parameters

v_max3 = 75; % maximum speed of platoon

v_min3 = 50; % minimum speed of platoon

% p_iniz3 = 0; %starting position of platoon

M_path3 = 6; % number of links included in the path of platoon 1

N_path3 = N_m(1,1) +  N_m(2,1) +  N_m(3,1) + N_m(4,1) + N_m(5,1)+ N_m(10,1) + 10; % number of sections included in the path of platoon 1

% delta_v = 0;

%% Freeway parameters

v_traffic3 = zeros(N_path3,K_fin3); %speed of traffic in platoon path

v_traffic3(1:N_m(1,1),:) = v_m1(2:N_m(1,1)+1,K_leader3+1+x-j2:K_finC+K_leader3);
v_traffic3(N_m(1,1)+ 1 : N_m(1,1)+ N_m(2,1) +1 ,:) = v_m2(:,K_leader3+1+x-j2:K_finC+K_leader3);
v_traffic3(N_m(1,1)+ N_m(2,1)+1 : N_m(1,1)+ N_m(2,1)+ N_m(3,1) +1 , :) = v_m3(:,K_leader3+1+x-j2:K_finC+K_leader3);
v_traffic3(N_m(1,1)+ N_m(2,1)+ N_m(3,1) +1 : N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1) +1 , :) = v_m4(:,K_leader3+1+x-j2:K_finC+K_leader3);
v_traffic3(N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1)+ 1 : N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1) + N_m(5,1) +1 , :) = v_m5(:,K_leader3+1+x-j2:K_finC+K_leader3);
v_traffic3(N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1)+ N_m(5,1)+ 1 : N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1) + N_m(5,1)+ N_m(10,1) +1 , :) = v_m10(:,K_leader3+1+x-j2:K_finC+K_leader3);
v_traffic3(N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1) + N_m(5,1)+ N_m(10,1) +2: N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1) + N_m(5,1)+ N_m(10,1) +5 , :) = v_m4(N_m(4,1)-2:N_m(4,1)+1,K_leader3+1+x-j2:K_finC+K_leader3);
v_traffic3(N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1) + N_m(5,1)+ N_m(10,1) +6: N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1) + N_m(5,1)+ N_m(10,1) +9 , :) = v_m5(N_m(5,1)-2:N_m(5,1)+1,K_leader3+1+x-j2:K_finC+K_leader3);
v_traffic3(N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1) + N_m(5,1)+ N_m(10,1) +10: N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1) + N_m(5,1)+ N_m(10,1) +11 , :) = v_m10(N_m(10,1):N_m(10,1)+1,K_leader3+1+x-j2:K_finC+K_leader3);



vMinLeader3 = zeros(N_path3,K_fin3); % lowest value between min speed and speed of traffic in each section at each time step
for k = 1:K_fin3

    for i = 1: N_path3
        if (v_min3 < v_traffic3(i,k))
        
            vMinLeader3(i,k) = v_min3;
        else
            vMinLeader3(i,k) = v_traffic3(i,k);
        end
    end
end

posiz_sez3 = zeros (1,N_path3); %initial position of each section belonging to the platoon path

for i = 2 : N_path3

    if i >= 2 && i<= N_m(1,1) + 1

        posiz_sez3(1,i) = (i-1) * Delta_M(1,1);

    elseif i > N_m(1,1) + 1 && i <= N_m(1,1) + N_m(2,1) +1

        posiz_sez3(1,i) = (i-1) * Delta_M(2,1);

    elseif i > N_m(1,1) + N_m(2,1) + 1 && i <= N_m(1,1) + N_m(2,1) + N_m(3,1) +1

        posiz_sez3(1,i) = (i-1) * Delta_M(3,1);

    elseif i > N_m(1,1) + N_m(2,1)+ N_m(3,1) + 1 && i <= N_m(1,1) + N_m(2,1) + N_m(3,1) + N_m(4,1) +1

        posiz_sez3(1,i) = (i-1) * Delta_M(4,1);

    elseif i > N_m(1,1) + N_m(2,1)+ N_m(3,1) + N_m(4,1) + 1 && i <= N_m(1,1) + N_m(2,1) + N_m(3,1) + N_m(4,1) + N_m(5,1) +1

        posiz_sez3(1,i) = (i-1) * Delta_M(5,1);

    elseif i > N_m(1,1) + N_m(2,1)+ N_m(3,1) + N_m(4,1)+ N_m(5,1) + 1 && i <= N_m(1,1) + N_m(2,1) + N_m(3,1) + N_m(4,1) + N_m(5,1)+ N_m(10,1) +1

        posiz_sez3(1,i) = (i-1) * Delta_M(10,1);

    else

        posiz_sez3(1,i) = (i-1) * Delta_M(10,1);
    end
end


%% Weights of the objective funtion

alpha_1 = 1;

alpha_2 = 0.001;

%% Other parameters

BigM = 100000;

epsilon = 0.001;

%% Platoon state variable

p3 = sdpvar(1,K_fin3); % position of the platoon

%% Control variable

v3 = sdpvar(1,K_fin3); % speed of the platoon

%% Ausiliary variables

y3 = binvar(N_path3,K_finC); % equal to one if the platoon is after the section i in k

w3 = binvar(N_path3,K_finC); % equal to one if the platoon is before the section i in k

lambda3 = binvar(N_path3,K_finC); % equal to one if the platoon is into section i in k



%% Objective function

Objective3 = 0;


% for k = 1: K_fin
% 
%     Objective = Objective + alpha_1 * ( p_bar - p(1,k_bar) )^2 ;
% end

Objective3 = Objective3 + alpha_1 * ( p_bar3 - p3(1,k_bar3) )^2 ;

for k = 1: (K_fin3) - 1

    Objective3 = Objective3 + alpha_2*(v3(1,k+1) - v3(1,k))^2;

end


%% constraints

%constraint (2) platoon position

ConstrPos13 = [];

for k = 1:K_fin3-1
    ConstrPos13 = [ ConstrPos13, p3(1,k+1) == p3(1,k) + v3(1,k)*(T)];
end

%constraint on the initial position of the platoon

ConstrPosIniz13 = [];

ConstrPosIniz13 =[ConstrPosIniz13, p3(1,1) == p_iniz3];

%constraint on the initial speed of the platoon

ConstrPosInizv3 = [];

ConstrPosInizv3 =[ConstrPosInizv3, v3(1,1) == v_iniz3];


%constraint (3) y part 1

ConstrYp1_leader3 = [];

for k = 1:K_fin3
    for i = 1: N_path3
        ConstrYp1_leader3 = [ ConstrYp1_leader3, (p3(1,k) - posiz_sez3(1,i)) + BigM*( 1 - y3(i,k) ) >= epsilon ];
    end
end


%constraint (4) y part 3

ConstrYp2_leader3 = [];

for k = 1:K_fin3
    for i = 1: N_path3
        ConstrYp2_leader3 = [ConstrYp2_leader3, (posiz_sez3(1,i) - p3(1,k)) + BigM * y3(i,k) >= 0];
    end
end


%constraint (5) w part 1

ConstrWp1_leader3 = [];

for k = 1:K_fin3
    for i = 1: N_path3 - 1
        ConstrWp1_leader3 = [ConstrWp1_leader3, (p3(1,k) - posiz_sez3(1,i+1)) + BigM * w3(i,k) >= epsilon ];
    end
end


%constraint (6) w part 2

ConstrWp2_leader3 = [];

for k = 1:K_fin3
    for i = 1: N_path3 - 1
        ConstrWp2_leader3 = [ConstrWp2_leader3, (posiz_sez3(1,i+1) - p3(1,k)) + BigM * (1- w3(i,k)) >= 0];
    end
end


%constraint (7) lambda part 1

Constrlambdap1_leader3 = [];

for k = 1:K_fin3
    for i = 1: N_path3
        Constrlambdap1_leader3 =  [Constrlambdap1_leader3,lambda3(i,k) <= y3(i,k)] ;
    end
end



%constraint (8) lambda part 2

Constrlambdap2_leader3 = [];

for k = 1:K_fin3
    for i = 1: N_path3
        Constrlambdap2_leader3 = [Constrlambdap2_leader3,lambda3(i,k) <= w3(i,k)];
    end
end

%constraint (9) lambda part 3

Constrlambdap3_leader3 = [];

for k = 1:K_fin3
    for i = 1: N_path3
        Constrlambdap3_leader3 = [Constrlambdap3_leader3, lambda3(i,k) >= y3(i,k) + w3(i,k) - 1];
    end
end


%constraint (10) traffic speed

ConstrSpeedTraf_leader3 = [];


for k = 2:K_fin3

    somma13 = 0;

    for i = 1: N_path3
        somma13 = somma13 + lambda3(i,k)*v_traffic3(i,k);
    end

    ConstrSpeedTraf_leader3 = [ConstrSpeedTraf_leader3, v3(1,k) <= somma13];

end

%constraint (11) speed min

ConstrSpeedPos_leader3=[];

for k = 1:K_fin3
    ConstrSpeedPos_leader3 = [ConstrSpeedPos_leader3, v3(1,k) >= 0];
end


%constraint (12) maximum speed of platoon

ConstrSpeedMax_leader3 = [];

for k = 1:K_fin3
    ConstrSpeedMax_leader3 = [ConstrSpeedMax_leader3, v3(1,k) <=  v_max3];
end

%constraint2 (13) minimum speed of platoon

ConstrSpeedMin_leader3 = [];

for k = 1:K_fin3

    somma23=0;

    for i = 1: N_path3
        somma23 = somma23 + lambda3(i,k)*vMinLeader3(i,k);
    end

    ConstrSpeedMin_leader3 = [ConstrSpeedMin_leader3, v3(1,k) >= somma23];

end

% %constraint (16) speed difference upper limit for initial speed
% 
% ConstrDeltaSpeedLeaderInizMax3 = [];
% 
%    ConstrDeltaSpeedLeaderInizMax3 = [ConstrDeltaSpeedLeaderInizMax3, (v3(1,1) - v_iniz3) <= delta_v];
% 
% 
% 
% %constraint (17) speed difference lower limit for initial speed
% 
% ConstrDeltaSpeedLeaderInizMin3 = [];
% 
%    ConstrDeltaSpeedLeaderInizMin3 = [ConstrDeltaSpeedLeaderInizMin3, (v3(1,1) - v_iniz3) >= -delta_v];


Constraints3=[ConstrPos13, ConstrPosIniz13, ConstrPosInizv3, ConstrYp1_leader3, ConstrYp2_leader3, ConstrWp1_leader3, ConstrWp2_leader3, Constrlambdap1_leader3,...
Constrlambdap2_leader3, Constrlambdap3_leader3, ConstrSpeedTraf_leader3, ConstrSpeedPos_leader3, ConstrSpeedMax_leader3, ConstrSpeedMin_leader3];

% Risolvo con Gurobi

options=sdpsettings('solver','gurobi');
optimize(Constraints3,Objective3,options);

% Risolvo con Yalmip

%optimize(Constraints,Objective);

% Scrivo la soluzioni in matrici matlab
%
solutionp3=value(p3);
solutionv3=value(v3);
solutiony3=value(y3);
solutionw3=value(w3);
solutionlambda3=value(lambda3);
optimum3=value(Objective3);
