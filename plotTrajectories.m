function figNum = plotTrajectories(traj, separators)
    t = traj.t;
    ti = t(1);
    tf = t(length(t));
    titles = ["pos","vel","acc", "jerk", "snap"];
    dstr = "";
    tstr = "";
    props = ["q","dq","ddq","dddq","ddddq"];
    n = length(fieldnames(traj))-1; %-1 to not count t
    dim = ceil(sqrt(n));
    figRef = figure;
    figNum = figRef.Number;
    for i = 1:n
        prop = props(i);
        if ~isfield(traj,prop)
            continue;
        end
        subplot(n,1,i);
        plot(t,traj.(prop));
        xlim([ti tf]);
        title(titles(i));
        xlabel("time (s)");
        
        if i > 1
            dstr = strcat(dstr, 'd');
        end
        if i == 2 
            tstr = '/s';
        end
        if (i>2) 
            tstr = strcat('/s^', num2str(i-1));
        end
        
        ylabel(strcat(dstr,'q ','(rad',tstr,')'));
        
        if nargin == 2
            for i=1:length(separators)
                xline(separators(i), ':');
            end
            
        end
    end 
end

