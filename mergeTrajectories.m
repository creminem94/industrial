function traj = mergeTrajectories(trajectories,props)
    if nargin == 1
        props = ["t","q","dq","ddq","dddq"];
    end
    n = length(trajectories);
    for j=1:length(props)
        prop = props(j);
        v = [];
        for i=1:n
            argTraj = trajectories(i);
            if ~isfield(argTraj,prop)
                continue;
            end
            newData = argTraj.(prop);
            if prop == "t" && i > 1
                newData = newData+v(length(v));
            end
            v = [v newData];
        end
        traj.(prop) = v;
    end
end

