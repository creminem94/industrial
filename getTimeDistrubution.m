function [tk] = getTimeDistrubution(points, metric, mu)
    n = length(points);
    if metric == "eq"
         dk = ones(1,n-1)*(1/(n-1));
    else
        norm = zeros(1,n-1);
        for k=1:n-1
           norm(k)=abs(points(k+1)-points(k))
        end
        switch metric     
            case "cord"
                dk = norm;
            case "centr"
                dk = sqrt(norm);
            case "power"
                if nargin < 3
                    mu = 1
                end
                dk = norm.^mu;
            otherwise
                error("select a valid metric: eq,cord,centr,power");
        end
    end
    d = sum(dk);
    tk = zeros(1,n);
    for k=2:n
       tk(k) = tk(k-1)+dk(k-1)/d;
    end
end

