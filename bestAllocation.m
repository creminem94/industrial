function allocation = bestAllocation(subtasks, W)
    % Weight factors
    Wda = W(1); Wpa = W(2);
    Wdk = W(3); Wpk = W(4);
    
    % Best allocation value
    bestJ = realmax;

    nTasks = length(subtasks);
    nAllocations = 2^(nTasks);
    for i = 0:nAllocations-1
        
        % Binary string corresponding to the allocation sequence
        bin = pad(dec2bin(i),nTasks,'left','0');

        J = 0;
        for j=1:nTasks
            if logical(str2num(bin(j))) % 1 -> Kuka
                Dk = subtasks(j).Dk;
                Pk = subtasks(j).Pk;
                Da = 0;
                Pa = 0;
            else                        % 0 -> ABB
                Da = subtasks(j).Da;
                Pa = subtasks(j).Pa;                
                Dk = 0;
                Pk = 0;
            end
            
            % Allocation equation
            J = J + (Wda*Da + Wpa*Pa + Wdk*Dk + Wpk*Pk);
        end
        
        if J < bestJ
            % Better allocation found
            bestJ = J;
            allocation = bin;
        end
        
    end
    
end

