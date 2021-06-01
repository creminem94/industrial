function allocation = bestAllocation(subtasks)
    nTasks = length(subtasks);
    nAllocations = 2^(nTasks);
    bestJ = realmax;
    for i = 0:nAllocations-1
        bin = pad(dec2bin(i),nTasks,'left','0');
%         j = 
        if j > bestJ
            bestJ = j;
            allocation = bin;
        end
    end
end

