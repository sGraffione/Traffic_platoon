clear all
% mpcverbosity off

x=0;
x1=0;
a3=0;
a1=0;
a2=0;
b=0;
c=0;


% Time of entrance of the L with respect to the model
K_leader1 = 200;
K_leader2 = 100;
K_leader3 = 0;
K_follower = 0;
K_follower1 = 0;

% Deadline of the arrival of the L
k_bar1 = 110;
K_fin1 = k_bar1 + 5;
K_finA = K_fin1; % original meeting time of the leader because K_fin1 changes when K_bar1 changes
k_bar2 = 110;
K_fin2 = k_bar2 + 5;
K_finB = K_fin2; % original meeting time of the leader because K_fin2 changes when K_bar2 changes
k_bar3 = 130;
K_fin3 = k_bar3 + 5;
K_finC = K_fin3; % original meeting time of the leader because K_fin3 changes when K_bar3 changes

% Initial position of the L and Fs for each part of the trip
p_iniz1 = 0;

p_iniz2 = 0;
p_iniz3 = 0;
p_iniz_follower2 = 0;
p_iniz_follower1 = 0;

% Actual speed of the leader
vLeader = zeros(1,K_fin1 + K_fin2 + K_fin3);
uLeader = zeros(1,K_fin1 + K_fin2 + K_fin3);

% Speed of traffic in the section of the leader
v_traffic_section_leader = zeros (1,K_fin1 + K_fin2 + K_fin3);

% Position of the L
p_L = zeros(1,K_fin1 + K_fin2 + K_fin3);

% the cells where the L is
cell_L = zeros(1,K_fin1 + K_fin2 + K_fin3);

k1 = 0; % timer

%% The first part of the leader's trip
% Run METANET and the problems for the first time
run Data_traffic.m
run Variables.m
run Traffic_model.m %METANET
% v_iniz1 = v_m1(2,1);
% v_iniz_follower1 =  60;
run Leader1Iniz.m % the first part of the leader's trip

for i=1:K_fin1 % to find out the meeting time for the first time
    if (solutionp1(1,i) >= 16.8 && solutionp1(1,i-1) < 16.8)
        A = i;
    end
end
k_bar_follower1 = A;
run Follower1Iniz.m % first Follower problem

% Actual speeds of F
vFollower1 = zeros(1,K_fin_follower1);


% Positions of the F
p_F1 = zeros(1,K_fin_follower1);
% p_F1(1,1) = 0.01;


% the cells where the F is
cell_F1 = zeros(1,K_fin_follower1);


% Receiding horizon: every 2 sections (every 1.4 km)

%the value of z at each time instant
% z1=zeros(1,K_fin1);
%%
clear all
load("ws1.mat");
mpcverbosity off

for x=1:K_finA-1

    k1 = k1 + 1; % timer that resets after each horizon

    if (p_L(1,x) == 0)
        cell_L(1,x) = 1;
    else
        cell_L(1,x) = ceil(p_L(1,x)/0.7);
    end

    %vLeader(1,x) = solutionv1(1,k1+1);

    if solutionz1 == 1
        if (x <= A)
            % if  (p_F(1,j) <= p_bar_follower)
            if (x == 1)
                cell_F1(1,x) = 1;
            else
                cell_F1(1,x) = ceil(p_F1(1,x)/0.7);
            end
            vFollower1(1,x) = solutionv_follower1(1,k1+1);
        end
    end

    if (x>1 && cell_L(1,x) > 1 && mod (cell_L(1,x-1),2)==0 && mod (cell_L(1,x),2)~=0 && cell_L(1,x)<=N_path1) % distance horizon equals to two cells
        a1=a1+1;
        % k1=k1+1;
        if  x <= A+1
            p_iniz_follower1 = p_F1(1,x);
            % if vFollower1(1,x) < v_traffic_follower1(cell_F1(1,x),k1)
            % v_iniz_follower1 = vFollower1(1,x);
            % else
            % v_iniz_follower1 = v_traffic_follower1(cell_F1(1,x),k1);
            % end
            v_iniz_follower1 = vFollower1(1,x-1);
        end

        p_iniz1 = p_L(1,x);
        if vLeader(1,x) < v_traffic1(cell_L(1,x),k1+1)
            v_iniz1 = vLeader(1,x);
        else
            v_iniz1 = v_traffic1(cell_L(1,x),k1+1);
        end

        % reducing the meeting time of the leader only for the optimisation problem
        k_bar1 = k_bar1 - k1;

        run Traffic_model.m %METANET
        run Leader1.m %Leader problem

        % If the follower decides to join the leader
        % && cell_F1(1,j)<=N_path_follower1 && p_F1(1,j-1)<p_bar_follower1

        %    if ( cell_F1(1,j) > 0 && solutionz1==1  && j <= A+1 && j>1 ) %before the leader reaches the meeting point
        if (p_F1(1,x) < p_bar_follower1 && solutionz1==1)
            b=b+1;
            % H = k_bar_follower1;
            for i=1:(K_fin1)
                if (i>1 && solutionp1(1,i) >= 16.8 && solutionp1(1,i-1) < 16.8 )
                    k_bar_follower1 = i;
                end
            end
                       %    if k_bar_follower1 ~= H-k1
           run Follower1.m %Follower problem
                       %    end
        end
        k1 = 0;
    end
    % if k1 == 0
    %     k1 = 1;
    % end

    %k_traffic is k + k of the entrance of the Leader platoon
    %  k_traffic = j + K_leader;
    if x==1
        vLeader(1,x:x+1) = solutionv1(1,k1+1);
        uLeader(1,x:x+1) = 0; % init control action for leader
    else
        % Implement the platoon microscopic control
        vLeader_desired(1,x) = solutionv1(1,k1+1);
        [stateL_temp,uLeader_temp] = decisionControlL([p_L(1,x) vLeader(1,x)]',[p_L(1,x-1) vLeader(1,x-1)]',uLeader(1,x),v_traffic1(cell_L(1,x),k1+1),vLeader_desired(1,x),10,cell_L(1,x));
        p_L(1,x+1) = stateL_temp(1);
        vLeader(1,x+1) = stateL_temp(2);
        uLeader(1,x+1) = uLeader_temp;
    end

    % if k1 == 0 && vLeader(1,x)>v_traffic1(cell_L(1,x),k1+1)
    %     vLeader(1,x)=v_traffic1(cell_L(1,x),k1+1);
    % elseif k1 ~= 0
    %     vLeader(1,x)=v_traffic1(cell_L(1,x),k1);
    % end
    % v_traffic_section_leader(1,x) = v_traffic1(cell_L(1,x),x);
    % vLeader(1,k) = min ([v1(1,k1) ; vTrafficLeader(cell_L(1,k),k_traffic)]); % vTrafficLeader is the real traffic speed on the leader's path, i.e. with noise
    %p_L(1,x+1) = p_L(1,x) + vLeader(1,x)*T; SOVRASCRITTA DA MPC DI SIMONE E ALESSANDRO

    if solutionz1 == 1
        if (x <= A)
            % if  (p_F(1,j) <= p_bar_follower)
            if (x == 1)
                cell_F1(1,x) = 1;
            else
                cell_F1(1,x) = ceil(p_F1(1,x)/0.7);
            end
            if k1==0
                vFollower1(1,x) = solutionv_follower1(1,k1+1);
            else
                vFollower1(1,x) = vFollower1(1,x) ;
            end

            % vFollower(1,k) = min ([v_follower(1,k1) ; vTrafficFollower(cell_L(1,k),k_traffic)]); % vTrafficFollower is the real traffic speed on the follower's path, i.e. with noise

            p_F1(1,x+1) = p_F1(1,x) + vFollower1(1,x)*T;

            if  (p_F1(1,x+1) >= p_bar_follower1)
                p_F1(1,x+1) = p_bar_follower1;
            end

        end
    end

    if (p_L(1,x+1) >= 16.8)
        j1 = x+1;
        break
    end
end

x = j1;
%% The second part of the leader's trip
% Run METANET and the problems for the second time
run Data_traffic.m
run Variables.m
run Traffic_model.m %METANET
v_iniz2 = vLeader(1,x-1);
% v_iniz_follower2 = 60;
n=1;
run Leader2.m % the second part of the leader's trip
% n=2;
vLeader(1,j1) = solutionv2(1,2);
p_L(1,j1+1) = p_L(1,j1) + vLeader(1,j1)*T;
if (p_L(1,x)-16.8 == 0)

    cell_L(1,x) = 1;

else

    cell_L(1,x) = ceil((p_L(1,x)-16.8)/0.7);

end
% calculate v leader at j1 and p leader at j1 + 1
for i=1:K_fin2 % to find out the meeting time for the first time
    if (solutionp2(1,i) >= 18.2 && solutionp2(1,i-1) < 18.2)
        B = i;
    end
end
k_bar_follower2 = B;
l=1;
h=0;
run Follower2Iniz.m % second Follower problem
l=0;
h=1;
vFollower2 = zeros(1,A+B);
p_F2 = zeros(1,A+B);
p_F2(1,j1) = 0.01;
cell_F2 = zeros(1,A+B);
cell_F2(1,j1) = 1;
% calculate p follower2 at j1 + 1 and v follower2 at j1
vFollower2(1,x) = solutionv_follower2(1,1);
p_F2(1,x+1) = p_F2(1,x) + vFollower2(1,x)*T;
% Receiding horizon: every 2 sections (every 1.4 km)

%the value of z at each time instant
% z2=zeros(1,K_fin2);


k1 = 0;

for x=j1+1:j1+K_finB

    % z2(1,j) = solutionz2;

    k1 = k1 + 1; % timer that resets after each horizon

    if (p_L(1,x)-16.8 == 0)

        cell_L(1,x) = 1;

    else

        cell_L(1,x) = ceil((p_L(1,x)-16.8)/0.7);

    end

    %k_traffic is k + k of the entrance of the Leader platoon
    %  k_traffic = j + K_leader;
    % if a2 == 0
    % vLeader(1,x) = solutionv2(1,k1+1);
    % elseif a2 ~= 0 && k1 == 1
    vLeader(1,x) = solutionv2(1,k1+1);%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % else
    %     vLeader(1,x) = solutionv2(1,k1);
    % end

    % v_traffic_section_leader(1,x) = v_traffic2(cell_L(1,x),x-j1+1);
    % vLeader(1,k) = min ([v1(1,k1) ; vTrafficLeader(cell_L(1,k),k_traffic)]); % vTrafficLeader is the real traffic speed on the leader's path, i.e. with noise
    if solutionz2 == 1
        if (x <= A+B)
            % if  (p_F(1,j) <= p_bar_follower)


            cell_F2(1,x) = ceil(p_F2(1,x)/0.7);



            vFollower2(1,x) = solutionv_follower2(1,k1+1);
            % vFollower(1,k) = min ([v_follower(1,k1) ; vTrafficFollower(cell_L(1,k),k_traffic)]); % vTrafficFollower is the real traffic speed on the follower's path, i.e. with noise
        end
    end

    if (cell_L(1,x) > 1 && mod (cell_L(1,x-1),2)==0 && mod (cell_L(1,x),2)~=0 && cell_L(1,x)<=N_path2) % distance horizon equals to two cells

        a2=a2+1;

        if  x <= A+B+1

            p_iniz_follower2 = p_F2(1,x);

            if vFollower2(1,x) < v_traffic_follower2(cell_F2(1,x),k1+1)
                v_iniz_follower2 = vFollower2(1,x);
                        else
                            v_iniz_follower2 = v_traffic_follower2(cell_F2(1,x),k1+1);
            end
            v_iniz_follower2 = vFollower2(1,x-1);

        end

        p_iniz2 = p_L(1,x)-16.8;
        % if vLeader(1,x) < v_traffic2(cell_L(1,x),k1+2)
        % v_iniz2 = vLeader(1,x);
        % else
        % v_iniz2 = v_traffic2(cell_L(1,x),k1+2);
        % end
        % v_iniz2 = min([vLeader(1,x); v_traffic2(cell_L(1,x),k1+2); v_traffic2(cell_L(1,x),k1+1)]);
        % v_iniz2 = solutionv2(1,k1);
        % if vLeader(1,x) < v_traffic2(cell_L(1,x),k1+1)
        v_iniz2 = vLeader(1,x);
        % else
        % v_iniz2 = v_traffic2(cell_L(1,x),k1+1);
        % end

        % reducing the meeting time of the leader only for the optimisation problem
        k_bar2 = k_bar2 - k1;


        % Time = 0;

        run Traffic_model.m %METANET
        run Leader2.m %Leader problem

        % If the follower decides to join the leader

        %    if ( cell_F2(1,j) > 0 && solutionz2==1 && cell_F2(1,j)<=N_path_follower2 && j <= A+B+1 && j>1 && p_F2(1,j-1)<p_bar_follower2) %before the leader reaches the meeting point
        if (p_F2(1,x) < p_bar_follower2 && solutionz2==1)
            c=c+1;
            for i=1:(K_fin2)
                if (i>1 && solutionp2(1,i) >= 18.2 && solutionp2(1,i-1) < 18.2 )
                    k_bar_follower2 = i;
                end
                       end

                       run Follower2.m %Follower problem

        end
        k1 = 0;

    end

    if k1==0
        vLeader(1,x) = solutionv2(1,k1+1);
    else
        vLeader(1,x) = vLeader(1,x);
    end

    % if k1==0
    % vLeader(1,x) = solutionv2(1,k1+2);
    % elseif a2 == 0
    %     vLeader(1,x) = solutionv2(1,k1+1);
    % else
    %     vLeader(1,x) = solutionv2(1,k1);
    % end

    p_L(1,x+1) = p_L(1,x) + vLeader(1,x)*T;



    if solutionz2 == 1
        if (x <= A+B)
            % if  (p_F(1,j) <= p_bar_follower)


            if (x == j1)

                cell_F2(1,x) = 1;

            else

                cell_F2(1,x) = ceil(p_F2(1,x)/0.7);

            end

            if k1==0
                vFollower2(1,x) = solutionv_follower2(1,k1+1);
            else
                vFollower2(1,x) = vFollower2(1,x) ;
            end


            % vFollower(1,k) = min ([v_follower(1,k1) ; vTrafficFollower(cell_L(1,k),k_traffic)]); % vTrafficFollower is the real traffic speed on the follower's path, i.e. with noise

            p_F2(1,x+1) = p_F2(1,x) + vFollower2(1,x)*T;

            if  (p_F2(1,x+1) >= p_bar_follower2)
                p_F2(1,x+1) = p_bar_follower2;
            end

        end
    end

    if (p_L(1,x+1) >= 18.2 + 16.8)
        j2 = x+1;
        break

    end


end



%% The third part of the leader's trip
% Run METANET and the problems for the first time
x=j2;
run Data_traffic.m
run Variables.m
run Traffic_model.m %METANET
v_iniz3 = vLeader(1,x-1);
run Leader3.m % the third part of the leader's trip
vLeader(1,j2) = solutionv3(1,1);
p_L(1,j2+1) = p_L(1,j2) + vLeader(1,j2)*T;
if (p_L(1,x)-16.8-18.2 == 0)

    cell_L(1,x) = 1;

else

    cell_L(1,x) = ceil((p_L(1,x)-16.8-18.2)/0.7);

end
% Receiding horizon: every 2 sections (every 1.4 km)

k1 = 0;

for x=j2+1:j2+K_finC

    k1 = k1 + 1; % timer that resets after each horizon

    if (p_L(1,x)-16.8-18.2 == 0)

        cell_L(1,x) = 1;

    else

        cell_L(1,x) = ceil((p_L(1,x)-16.8-18.2)/0.7);

    end

    %k_traffic is k + k of the entrance of the Leader platoon
    %  k_traffic = j + K_leader;
    vLeader(1,x) = solutionv3(1,k1+1);
    % v_traffic_section_leader(1,x) = v_traffic3(cell_L(1,x),x-j2+1);
    % vLeader(1,k) = min ([v1(1,k1) ; vTrafficLeader(cell_L(1,k),k_traffic)]); % vTrafficLeader is the real traffic speed on the leader's path, i.e. with noise


    if (cell_L(1,x) > 1 && mod (cell_L(1,x-1),2)==0 && mod (cell_L(1,x),2)~=0 && cell_L(1,x)<=N_path3) % distance horizon equals to two cells

        a3=a3+1;

        p_iniz3 = p_L(1,x)-16.8-18.2;
        % if vLeader(1,x) < v_traffic3(cell_L(1,x),k1+1)
        v_iniz3 = vLeader(1,x);
        % else
        % v_iniz3 = v_traffic3(cell_L(1,x),k1+1);
        % end


        % reducing the meeting time of the leader only for the optimisation problem
        k_bar3 = k_bar3 - k1;

        k1 = 0;

        run Traffic_model.m %METANET
        run Leader3.m %Leader problem

    end

    if k1==0
        vLeader(1,x) = solutionv3(1,k1+1);
    else
        vLeader(1,x) = vLeader(1,x);
    end

    p_L(1,x+1) = p_L(1,x) + vLeader(1,x)*T;

    if (p_L(1,x+1) >= 57.4)
        j3 = x+1;
        break

    end

end



% %% Run METANET and the problems for the first time
% run Data_traffic.m
% run Variables.m
% run Traffic_model.m %METANET
%
% run Leader1.m
% for i=1:k_bar1 % to find out the meeting time for the second follower
%    if (solutionp1(1,i) >= 16.1 && solutionp1(1,i-1) < 16.1)
%    k_bar_follower1 = i;
%    end
% end
% run Follower1.m
%
%
% run Leader2.m
% for i=1:k_bar2 % to find out the meeting time for the second follower
%    if (solutionp2(1,i) >= 16.1 && solutionp2(1,i-1) < 16.1)
%    k_bar_follower2 = i;
%    end
% end
% run Follower2.m
%
%
% run Leader3.m