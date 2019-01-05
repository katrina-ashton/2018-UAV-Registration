function [a,u] = rotm2aa(R)
a = acos((trace(R)-1)/2);
if(any(any(R-R'))) 
    u = [R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2)];
else 
    [V,D] = eig(R);
    for i=1:3
        if V(i)==1
            u = D(:,i);
        end
    end
end
end