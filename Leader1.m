

%%% The leader platoon starts its journey starting from section 1 of link m1
%%% its destination is at the end of section 4 of link m10

%% Data

%% Meeting point parameters

p_bar1 = N_m(1,1)*Delta_M(1,1) + N_m(2,1)*Delta_M(2,1) + N_m(3,1)*Delta_M(3,1) + N_m(4,1)*Delta_M(4,1) + N_m(5,1)*Delta_M(5,1) + N_m(10,1)*Delta_M(10,1) + 3*Delta_M(10,1); % position of the meeting point

% k_bar1 = 120; % is the time step in which the meeting should occur

Delta_k = 5; %

K_fin1 = k_bar1 + Delta_k;

%% Platoon parameters

v_max1 = 75; % maximum speed of platoon

v_min1 = 50; % minimum speed of platoon

% p_iniz1 = 0; %starting position of platoon

M_path1 = 6; % number of links included in the path of platoon 1

N_path1 = N_m(1,1) +  N_m(2,1) +  N_m(3,1) + N_m(4,1) + N_m(5,1)+ N_m(10,1) + 3; % number of sections included in the path of platoon 1

delta_v = 0.2;

%% Freeway parameters

v_traffic1 = zeros(N_path1,K_fin1); %speed of traffic in platoon path

v_traffic1(1:N_m(1,1),:) = v_m1(2:N_m(1,1)+1,K_leader1+1+x:K_finA+K_leader1);
v_traffic1(N_m(1,1)+ 1 : N_m(1,1)+ N_m(2,1) +1 ,:) = v_m2(:,K_leader1+1+x:K_finA+K_leader1);
v_traffic1(N_m(1,1)+ N_m(2,1)+1 : N_m(1,1)+ N_m(2,1)+ N_m(3,1) +1 , :) = v_m3(:,K_leader1+1+x:K_finA+K_leader1);
v_traffic1(N_m(1,1)+ N_m(2,1)+ N_m(3,1) +1 : N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1) +1 , :) = v_m4(:,K_leader1+1+x:K_finA+K_leader1);
v_traffic1(N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1)+ 1 : N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1) + N_m(5,1) +1 , :) = v_m5(:,K_leader1+1+x:K_finA+K_leader1);
v_traffic1(N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1)+ N_m(5,1)+ 1 : N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1) + N_m(5,1)+ N_m(10,1) +1 , :) = v_m10(:,K_leader1+1+x:K_finA+K_leader1);
v_traffic1(N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1) + N_m(5,1)+ N_m(10,1) +2 : N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1) + N_m(5,1)+ N_m(10,1) +3 , :) = v_m10(N_m(10,1) : N_m(10,1)+1,K_leader1+1+x:K_finA+K_leader1);

vMinLeader = zeros(N_path1,K_fin1); % lowest value between min speed and speed of traffic in each section at each time step
for k = 1:K_fin1

    for i = 1: N_path1
        if (v_min1 < v_traffic1(i,k))
        
            vMinLeader(i,k) = v_min1;
        else
            vMinLeader(i,k) = v_traffic1(i,k);
        end
    end
end

posiz_sez1 = zeros (1,N_path1); %initial position of each section belonging to the platoon path

for i = 2 : N_path1

    if i >= 2 && i<= N_m(1,1) + 1

        posiz_sez1(1,i) = (i-1) * Delta_M(1,1);

    elseif i > N_m(1,1) + 1 && i <= N_m(1,1) + N_m(2,1) +1

        posiz_sez1(1,i) = (i-1) * Delta_M(2,1);

    elseif i > N_m(1,1) + N_m(2,1) + 1 && i <= N_m(1,1) + N_m(2,1) + N_m(3,1) +1

        posiz_sez1(1,i) = (i-1) * Delta_M(3,1);

    elseif i > N_m(1,1) + N_m(2,1)+ N_m(3,1) + 1 && i <= N_m(1,1) + N_m(2,1) + N_m(3,1) + N_m(4,1) +1

        posiz_sez1(1,i) = (i-1) * Delta_M(4,1);

    elseif i > N_m(1,1) + N_m(2,1)+ N_m(3,1) + N_m(4,1) + 1 && i <= N_m(1,1) + N_m(2,1) + N_m(3,1) + N_m(4,1) + N_m(5,1) +1

        posiz_sez1(1,i) = (i-1) * Delta_M(5,1);

    elseif i > N_m(1,1) + N_m(2,1)+ N_m(3,1) + N_m(4,1)+ N_m(5,1) + 1 && i <= N_m(1,1) + N_m(2,1) + N_m(3,1) + N_m(4,1) + N_m(5,1)+ N_m(10,1) +1

        posiz_sez1(1,i) = (i-1) * Delta_M(10,1);

    else

        posiz_sez1(1,i) = (i-1) * Delta_M(10,1);
    end
end


%% Weights of the objective funtion

alpha_1 = 1;

alpha_2 = 0.001;

%% Other parameters

BigM = 100000;

epsilon = 0.001;

%% Platoon state variable

p1 = sdpvar(1,K_fin1); % position of the platoon

%% Control variable

v1 = sdpvar(1,K_fin1); % speed of the platoon

%% Ausiliary variables

y1 = binvar(N_path1,K_finA); % equal to one if the platoon is after the section i in k

w1 = binvar(N_path1,K_finA); % equal to one if the platoon is before the section i in k

lambda1 = binvar(N_path1,K_finA); % equal to one if the platoon is into section i in k



%% Objective function

Objective1 = 0;


% for k = 1: K_fin
% 
%     Objective = Objective + alpha_1 * ( p_bar - p(1,k_bar) )^2 ;
% end

Objective1 = Objective1 + alpha_1 * ( p_bar1 - p1(1,k_bar1) )^2 ;

for k = 1: (K_fin1) - 1

    Objective1 = Objective1 + alpha_2*(v1(1,k+1) - v1(1,k))^2;

end


%% constraints

%constraint (2) platoon position

ConstrPos1 = [];

for k = 1:K_fin1-1
    ConstrPos1 = [ ConstrPos1, p1(1,k+1) == p1(1,k) + v1(1,k)*(T)];
end

%constraint on the initial position of the platoon

ConstrPosIniz1 = [];

ConstrPosIniz1 =[ConstrPosIniz1, p1(1,1) == p_iniz1];

%constraint on the initial speed of the platoon

ConstrPosInizv1 = [];

ConstrPosInizv1 =[ConstrPosInizv1, v1(1,1) == v_iniz1];


%constraint (3) y part 1

ConstrYp1_leader = [];

for k = 1:K_fin1
    for i = 1: N_path1
        ConstrYp1_leader = [ ConstrYp1_leader, (p1(1,k) - posiz_sez1(1,i)) + BigM*( 1 - y1(i,k) ) >= epsilon ];
    end
end


%constraint (4) y part 3

ConstrYp2_leader = [];

for k = 1:K_fin1
    for i = 1: N_path1
        ConstrYp2_leader = [ConstrYp2_leader, (posiz_sez1(1,i) - p1(1,k)) + BigM * y1(i,k) >= 0];
    end
end


%constraint (5) w part 1

ConstrWp1_leader = [];

for k = 1:K_fin1
    for i = 1: N_path1 - 1
        ConstrWp1_leader = [ConstrWp1_leader, (p1(1,k) - posiz_sez1(1,i+1)) + BigM * w1(i,k) >= epsilon ];
    end
end


%constraint (6) w part 2

ConstrWp2_leader = [];

for k = 1:K_fin1
    for i = 1: N_path1 - 1
        ConstrWp2_leader = [ConstrWp2_leader, (posiz_sez1(1,i+1) - p1(1,k)) + BigM * (1- w1(i,k)) >= 0];
    end
end


%constraint (7) lambda part 1

Constrlambdap1_leader = [];

for k = 1:K_fin1
    for i = 1: N_path1
        Constrlambdap1_leader =  [Constrlambdap1_leader,lambda1(i,k) <= y1(i,k)] ;
    end
end



%constraint (8) lambda part 2

Constrlambdap2_leader = [];

for k = 1:K_fin1
    for i = 1: N_path1
        Constrlambdap2_leader = [Constrlambdap2_leader,lambda1(i,k) <= w1(i,k)];
    end
end

%constraint (9) lambda part 3

Constrlambdap3_leader = [];

for k = 1:K_fin1
    for i = 1: N_path1
        Constrlambdap3_leader = [Constrlambdap3_leader, lambda1(i,k) >= y1(i,k) + w1(i,k) - 1];
    end
end


%constraint (10) traffic speed

ConstrSpeedTraf_leader = [];


for k = 2:K_fin1

    somma1 = 0;

    for i = 1: N_path1
        somma1 = somma1 + lambda1(i,k)*v_traffic1(i,k);
    end

    ConstrSpeedTraf_leader = [ConstrSpeedTraf_leader, v1(1,k) <= somma1];

end

%constraint (11) speed min

ConstrSpeedPos_leader=[];

for k = 1:K_fin1
    ConstrSpeedPos_leader = [ConstrSpeedPos_leader, v1(1,k) >= 0];
end


%constraint (12) maximum speed of platoon

ConstrSpeedMax_leader = [];

for k = 1:K_fin1
    ConstrSpeedMax_leader = [ConstrSpeedMax_leader, v1(1,k) <=  v_max1];
end

%constraint2 (13) minimum speed of platoon

ConstrSpeedMin_leader = [];

for k = 1:K_fin1

    somma2=0;

    for i = 1: N_path1
        somma2 = somma2 + lambda1(i,k)*vMinLeader(i,k);
    end

    ConstrSpeedMin_leader = [ConstrSpeedMin_leader, v1(1,k) >= somma2];

end

%constraint (16) speed difference upper limit for initial speed

% ConstrDeltaSpeedLeaderInizMax1 = [];
% 
%    ConstrDeltaSpeedLeaderInizMax1 = [ConstrDeltaSpeedLeaderInizMax1, (v1(1,1) - v_iniz1) <= delta_v];



%constraint (17) speed difference lower limit for initial speed

% ConstrDeltaSpeedLeaderInizMin1 = [];
% 
%    ConstrDeltaSpeedLeaderInizMin1 = [ConstrDeltaSpeedLeaderInizMin1, (v1(1,1) - v_iniz1) >= -delta_v];


Constraints1=[ConstrPos1, ConstrPosIniz1, ConstrPosInizv1, ConstrYp1_leader, ConstrYp2_leader, ConstrWp1_leader, ConstrWp2_leader, Constrlambdap1_leader,...
Constrlambdap2_leader, Constrlambdap3_leader, ConstrSpeedTraf_leader, ConstrSpeedPos_leader, ConstrSpeedMax_leader, ConstrSpeedMin_leader];

% Risolvo con Gurobi

options=sdpsettings('solver','gurobi');
optimize(Constraints1,Objective1,options);

% Risolvo con Yalmip

%optimize(Constraints,Objective);

% Scrivo la soluzioni in matrici matlab
%
solutionp1=value(p1);
solutionv1=value(v1);
solutiony1=value(y1);
solutionw1=value(w1);
solutionlambda1=value(lambda1);
optimum1=value(Objective1);
