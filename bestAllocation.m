function allocation = bestAllocation(subtasks)
    nTasks = length(subtasks);
    nAllocations = 2^(nTasks);
    bestJ = realmax;
    for i = 0:nAllocations-1
        bin = pad(dec2bin(i),nTasks,'left','0');

        J = 0;
        for j=1:nTasks
            if logical(str2num(bin(j))) % 1 -> Kuka
                Di = subtasks(j).Dk;
                Pi = subtasks(j).Pk;
            else                        % 0 -> ABB
                Di = subtasks(j).Da;
                Pi = subtasks(j).Pa;                
            end
            J = J + Di + Pi;
        end
        
        if J < bestJ
            bestJ = J;
            allocation = bin;
        end
    end
end

