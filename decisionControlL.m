function [state] = decisionControlL(state,velTraffic,velRef,TsTraffic,currSec)
    velTraffic = velTraffic/3.6;
    state(1) = state(1) * 1000;
    state(2) = state(2)/3.6;
    velRef = velRef/3.6;

    Ts = 0.1;
    Ca = 9;
    Cb = 0.06;
    Cc = 0.023;
    W = 10;
    Wp = W;

    v = 100/3.6;
    f_max = Ca+Cb*v+Cc*v^2;
    f_min = Ca;

    Hp = 15;
    Hc = 5;

    A = [0 1
        0 -(Cb+2*Cc*state(2))/Wp];
    B = [0; 1/Wp];
    C = eye(2);
    D = 0;

    sys = ss(A,B,C,D);
    sysd = c2d(sys,Ts);
    A = sysd.A;
    B = sysd.B;


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
    state(1) = state(1)/1000;
    state(2) = state(2)*3.6;
end