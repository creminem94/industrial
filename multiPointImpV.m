function traj = multiPointImpV(points, Ts)
    %points is a matrix with 3 rows defining time,position and velocity at
    %each point
    nPoints = size(points,2);
    q = [];
    dq = [];
    ddq = [];
    dddq = [];
    t = [];
    for k=1:nPoints-1
        tk = points(1,k);
        tkp1 = points(1,k+1);
        tSlot = tk:Ts:tkp1;
        tVar = tSlot-tk;
        Tk = tkp1-tk;
        qk = points(2,k);
        qkp1 = points(2,k+1);
        dqk = points(3,k);
        dqkp1 = points(3,k+1);
        a0 = qk;
        a1 = dqk;
        a2 = 1/Tk*(3*(qkp1-qk)/Tk-2*dqk-dqkp1);
        a3 = 1/Tk^2*(2*(qk-qkp1)/Tk+dqk+dqkp1);
        P = a3*tVar.^3+a2*tVar.^2+a1*tVar+a0;
        dP = 3*a3*tVar.^2+2*a2*tVar+a1;
        ddP = 6*a3*tVar+2*a2;
        dddP = ones(1,length(tSlot))*6*a3;
        t = [t tSlot];
        q = [q P];
        dq = [dq dP];
        ddq = [ddq ddP];
        dddq = [dddq dddP];
    end
    traj.t = t;
    traj.q = q;
    traj.dq = dq;
    traj.ddq = ddq;
    traj.dddq = dddq;
end

