function [state] = decisionControlL(state,velTraffic,velRef,TsTraffic,currSec)
    %%%% PARAMETERS: 
    % state: vehicle's state
    % velTraffic: current traffic velocity (upper limit) -- current and following cell
    % velRef: reference velocity according to the optimization algorithm
    % TsTraffic: traffic sampling time
    % currSec: section which currently contains the vehicle

    % Convert in the correct unit of measure for the algorithm
    velTraffic = velTraffic/3.6;
    state(1) = state(1) * 1000;
    state(2) = state(2)/3.6;
    velRef = velRef/3.6;
    % System's variables
    Ts = 0.1;
    Ca = 9;
    Cb = 0.06;
    Cc = 0.023;
    W = 10;
    Wp = W;
    v = 100/3.6;
    f_max = Ca+Cb*v+Cc*v^2;
    f_min = Ca;

    % System's matrices
    A = [0 1
        0 -(Cb+2*Cc*state(2))/Wp];
    B = [0; 1/Wp];
    C = eye(2);
    D = 0;
    % Create the system
    sys = ss(A,B,C,D);
    sysd = c2d(sys,Ts);
    A = sysd.A;
    B = sysd.B;
    
    % Design the MPC
    Hp = 15;
    Hc = 5;
    mpcobj = mpc(sysd,Ts);

    mpcobj.PredictionHorizon = Hp;
    mpcobj.ControlHorizon = Hc;

    mpcobj.MV.Min = 0;
    mpcobj.MV.Max = f_max;

    mpcobj.MV.RateMin = -f_max*Ts;
    mpcobj.MV.RateMax = f_max*Ts;

    mpcobj.OutputVariables(1).Min = -inf;
    mpcobj.OutputVariables(2).Min = 0;
    mpcobj.OutputVariables(1).Max = inf;
    mpcobj.OutputVariables(2).Max = velTraffic(1,1);

    mpcobj.Weights.OutputVariables = [0.001 1];

    xc = mpcstate(mpcobj);

    pRef = state(1) + velRef*TsTraffic;
    xRef = [pRef; velRef];
    %dstate = state - xRef;
    % Control loop for one traffic sampling instant (Traffic POV)
    % Each iteration of the loop is one platoon sampling instant (Platoon POV)
    for k = 1:TsTraffic/Ts
        u(k) = mpcmove(mpcobj,xc,state',xRef);
        state = A*state+B*u(k); % TODO mettere non lineare
%         state = dstate + xRef;
        if ceil(state(1)/700)>currSec
           mpcobj.OutputVariables(2).Max = [velTraffic(2,1)];
        end

        A = [0 1
        0 -(Cb+2*Cc*state(2))/Wp];
        B = [0; 1/Wp];
        
        sys = ss(A,B,C,D);
        sysd = c2d(sys,Ts);
        A = sysd.A;
        B = sysd.B;
    end
    % Re-convert results to be coherent with traffic units of measure
    state(1) = state(1)/1000;
    state(2) = state(2)*3.6;
end