

%%% The leader platoon starts its journey starting from section 1 of link m1
%%% its destination is at the end of section 4 of link m10

%% Data

%% Meeting point parameters

p_bar2 = N_m(1,1)*Delta_M(1,1) + N_m(2,1)*Delta_M(2,1) + N_m(3,1)*Delta_M(3,1) + N_m(4,1)*Delta_M(4,1) + N_m(5,1)*Delta_M(5,1) + N_m(10,1)*Delta_M(10,1) + 4*Delta_M(10,1); % position of the meeting point

% k_bar2 = 120; % is the time step in which the meeting should occur

Delta_k = 5; %

K_fin2 = k_bar2 + Delta_k;

%% Platoon parameters

v_max2 = 75; % maximum speed of platoon

v_min2 = 50; % minimum speed of platoon

% p_iniz2 = 0; %starting position of platoon

M_path2 = 6; % number of links included in the path of platoon 1

N_path2 = N_m(1,1) +  N_m(2,1) +  N_m(3,1) + N_m(4,1) + N_m(5,1)+ N_m(10,1) + 4; % number of sections included in the path of platoon 1

% delta_v = 0.2;

%% Freeway parameters

v_traffic2 = zeros(N_path2,K_fin2); %speed of traffic in platoon path

v_traffic2(1:N_m(1,1),:) = v_m1(2:N_m(1,1)+1,K_leader2+x-j1+n:K_finB+K_leader2);
v_traffic2(N_m(1,1)+ 1 : N_m(1,1)+ N_m(2,1) +1 ,:) = v_m2(:,K_leader2+x-j1+n:K_finB+K_leader2);
v_traffic2(N_m(1,1)+ N_m(2,1)+1 : N_m(1,1)+ N_m(2,1)+ N_m(3,1) +1 , :) = v_m3(:,K_leader2+x-j1+n:K_finB+K_leader2);
v_traffic2(N_m(1,1)+ N_m(2,1)+ N_m(3,1) +1 : N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1) +1 , :) = v_m4(:,K_leader2+x-j1+n:K_finB+K_leader2);
v_traffic2(N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1)+ 1 : N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1) + N_m(5,1) +1 , :) = v_m5(:,K_leader2+x-j1+n:K_finB+K_leader2);
v_traffic2(N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1)+ N_m(5,1)+ 1 : N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1) + N_m(5,1)+ N_m(10,1) +1 , :) = v_m10(:,K_leader2+x-j1+n:K_finB+K_leader2);
v_traffic2(N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1) + N_m(5,1)+ N_m(10,1) +2: N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1) + N_m(5,1)+ N_m(10,1) +5 , :) = v_m10(N_m(10,1)-2:N_m(10,1)+1,K_leader2+x-j1+n:K_finB+K_leader2);

vMinLeader2 = zeros(N_path2,K_fin2); % lowest value between min speed and speed of traffic in each section at each time step
for k = 1:K_fin2-1

    for i = 1: N_path2
        if (v_min2 < v_traffic2(i,k))
        
            vMinLeader2(i,k) = v_min2;
        else
            vMinLeader2(i,k) = v_traffic2(i,k);
        end
    end
end

posiz_sez2 = zeros (1,N_path2); %initial position of each section belonging to the platoon path

for i = 2 : N_path2

    if i >= 2 && i<= N_m(1,1) + 1

        posiz_sez2(1,i) = (i-1) * Delta_M(1,1);

    elseif i > N_m(1,1) + 1 && i <= N_m(1,1) + N_m(2,1) +1

        posiz_sez2(1,i) = (i-1) * Delta_M(2,1);

    elseif i > N_m(1,1) + N_m(2,1) + 1 && i <= N_m(1,1) + N_m(2,1) + N_m(3,1) +1

        posiz_sez2(1,i) = (i-1) * Delta_M(3,1);

    elseif i > N_m(1,1) + N_m(2,1)+ N_m(3,1) + 1 && i <= N_m(1,1) + N_m(2,1) + N_m(3,1) + N_m(4,1) +1

        posiz_sez2(1,i) = (i-1) * Delta_M(4,1);

    elseif i > N_m(1,1) + N_m(2,1)+ N_m(3,1) + N_m(4,1) + 1 && i <= N_m(1,1) + N_m(2,1) + N_m(3,1) + N_m(4,1) + N_m(5,1) +1

        posiz_sez2(1,i) = (i-1) * Delta_M(5,1);

    elseif i > N_m(1,1) + N_m(2,1)+ N_m(3,1) + N_m(4,1)+ N_m(5,1) + 1 && i <= N_m(1,1) + N_m(2,1) + N_m(3,1) + N_m(4,1) + N_m(5,1)+ N_m(10,1) +1

        posiz_sez2(1,i) = (i-1) * Delta_M(10,1);

    else

        posiz_sez2(1,i) = (i-1) * Delta_M(10,1);
    end
end


%% Weights of the objective funtion

alpha_1 = 1;

alpha_2 = 0.001;

%% Other parameters

BigM = 100000;

epsilon = 0.001;

%% Platoon state variable

p2 = sdpvar(1,K_fin2); % position of the platoon

%% Control variable

v2 = sdpvar(1,K_fin2); % speed of the platoon

%% Ausiliary variables

y2 = binvar(N_path2,K_finB); % equal to one if the platoon is after the section i in k

w2 = binvar(N_path2,K_finB); % equal to one if the platoon is before the section i in k

lambda2 = binvar(N_path2,K_finB); % equal to one if the platoon is into section i in k



%% Objective function

Objective2 = 0;


% for k = 1: K_fin
% 
%     Objective = Objective + alpha_1 * ( p_bar - p(1,k_bar) )^2 ;
% end

Objective2 = Objective2 + alpha_1 * ( p_bar2 - p2(1,k_bar2) )^2 ;

for k = 1: (K_fin2) - 1

    Objective2 = Objective2 + alpha_2*(v2(1,k+1) - v2(1,k))^2;

end


%% constraints

%constraint (2) platoon position

ConstrPos12 = [];

for k = 1:K_fin2-1
    ConstrPos12 = [ ConstrPos12, p2(1,k+1) == p2(1,k) + v2(1,k)*(T)];
end

%constraint on the initial position of the platoon

ConstrPosIniz12 = [];

ConstrPosIniz12 =[ConstrPosIniz12, p2(1,1) == p_iniz2];

%constraint on the initial speed of the platoon

ConstrPosInizv2 = [];

ConstrPosInizv2 =[ConstrPosInizv2, v2(1,1) == v_iniz2];


%constraint (3) y part 1

ConstrYp1_leader2 = [];

for k = 1:K_fin2
    for i = 1: N_path2
        ConstrYp1_leader2 = [ ConstrYp1_leader2, (p2(1,k) - posiz_sez2(1,i)) + BigM*( 1 - y2(i,k) ) >= epsilon ];
    end
end


%constraint (4) y part 3

ConstrYp2_leader2 = [];

for k = 1:K_fin2
    for i = 1: N_path2
        ConstrYp2_leader2 = [ConstrYp2_leader2, (posiz_sez2(1,i) - p2(1,k)) + BigM * y2(i,k) >= 0];
    end
end


%constraint (5) w part 1

ConstrWp1_leader2 = [];

for k = 1:K_fin2
    for i = 1: N_path2 - 1
        ConstrWp1_leader2 = [ConstrWp1_leader2, (p2(1,k) - posiz_sez2(1,i+1)) + BigM * w2(i,k) >= epsilon ];
    end
end


%constraint (6) w part 2

ConstrWp2_leader2 = [];

for k = 1:K_fin2
    for i = 1: N_path2 - 1
        ConstrWp2_leader2 = [ConstrWp2_leader2, (posiz_sez2(1,i+1) - p2(1,k)) + BigM * (1- w2(i,k)) >= 0];
    end
end


%constraint (7) lambda part 1

Constrlambdap1_leader2 = [];

for k = 1:K_fin2
    for i = 1: N_path2
        Constrlambdap1_leader2 =  [Constrlambdap1_leader2,lambda2(i,k) <= y2(i,k)] ;
    end
end



%constraint (8) lambda part 2

Constrlambdap2_leader2 = [];

for k = 1:K_fin2
    for i = 1: N_path2
        Constrlambdap2_leader2 = [Constrlambdap2_leader2,lambda2(i,k) <= w2(i,k)];
    end
end

%constraint (9) lambda part 3

Constrlambdap3_leader2 = [];

for k = 1:K_fin2
    for i = 1: N_path2
        Constrlambdap3_leader2 = [Constrlambdap3_leader2, lambda2(i,k) >= y2(i,k) + w2(i,k) - 1];
    end
end


%constraint (10) traffic speed

ConstrSpeedTraf_leader2 = [];


for k = 2:K_fin2

    somma12 = 0;

    for i = 1: N_path2
        somma12 = somma12 + lambda2(i,k)*v_traffic2(i,k);
    end

    ConstrSpeedTraf_leader2 = [ConstrSpeedTraf_leader2, v2(1,k) <= somma12];

end

%constraint (11) speed min

ConstrSpeedPos_leader2=[];

for k = 1:K_fin2
    ConstrSpeedPos_leader2 = [ConstrSpeedPos_leader2, v2(1,k) >= 0];
end


%constraint (12) maximum speed of platoon

ConstrSpeedMax_leader2 = [];

for k = 1:K_fin2
    ConstrSpeedMax_leader2 = [ConstrSpeedMax_leader2, v2(1,k) <=  v_max2];
end

%constraint2 (13) minimum speed of platoon

ConstrSpeedMin_leader2 = [];

for k = 1:K_fin2

    somma22=0;

    for i = 1: N_path2
        somma22 = somma22 + lambda2(i,k)*vMinLeader2(i,k);
    end

    ConstrSpeedMin_leader2 = [ConstrSpeedMin_leader2, v2(1,k) >= somma22];

end

% %constraint (16) speed difference upper limit for initial speed
% 
% ConstrDeltaSpeedLeaderInizMax2 = [];
% 
%    ConstrDeltaSpeedLeaderInizMax2 = [ConstrDeltaSpeedLeaderInizMax2, (v2(1,1) - v_iniz2) <= delta_v];
% 
% 
% 
% %constraint (17) speed difference lower limit for initial speed
% 
% ConstrDeltaSpeedLeaderInizMin2 = [];
% 
%    ConstrDeltaSpeedLeaderInizMin2 = [ConstrDeltaSpeedLeaderInizMin2, (v2(1,1) - v_iniz2) >= -delta_v];


Constraints2=[ConstrPos12, ConstrPosIniz12, ConstrPosInizv2, ConstrYp1_leader2, ConstrYp2_leader2, ConstrWp1_leader2, ConstrWp2_leader2, Constrlambdap1_leader2,...
Constrlambdap2_leader2, Constrlambdap3_leader2, ConstrSpeedTraf_leader2, ConstrSpeedPos_leader2, ConstrSpeedMax_leader2, ConstrSpeedMin_leader2];

% Risolvo con Gurobi

options=sdpsettings('solver','gurobi');
optimize(Constraints2,Objective2,options);

% Risolvo con Yalmip

%optimize(Constraints,Objective);

% Scrivo la soluzioni in matrici matlab
%
solutionp2=value(p2);
solutionv2=value(v2);
solutiony2=value(y2);
solutionw2=value(w2);
solutionlambda2=value(lambda2);
optimum2=value(Objective2);
