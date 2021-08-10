function traj = mergeTrajectories(trajectories,props)
    if nargin == 1
        props = ["t","q","dq","ddq","dddq"];
    end
    n = length(trajectories);
    sep = [];
    for j=1:length(props)
        prop = props(j);
        v = [];
        for i=1:n
            argTraj = trajectories(i);
            if ~isfield(argTraj,prop)
                continue;
            end
            v = [v argTraj.(prop)];
            sep(i) = length(v);
        end
        traj.(prop) = v;
    end
end

