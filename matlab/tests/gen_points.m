function [p] = gen_points(N, pmin, pmax, rmin, E1, order)
max_iter = 200000;

pass = false;

while(~pass)
    p = [];
    p(:,1) = (pmin + (pmax-pmin).*rand(1,3))';
    for n = 2:N
        tries = 0;
        pass = false;
        while(~pass && tries <= max_iter)
            candidate = (pmin + (pmax-pmin).*rand(1,3))';
            diff = E1(:,:,n)*(p - candidate);
            dist = (sum(diff.^order(n),1)).^(1 / order(n));

            if(dist > rmin(n))
                p(:,n) = candidate;
                pass = true;
            end
            tries = tries + 1;
        end
        if (tries > max_iter)
            break;
        end
    end
end
p = reshape(p, 1, 3, N);
