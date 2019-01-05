function [b,a,g] = rotm2euler(R)
b = atan2(sqrt(R(3,1)^2 +R(3,2)^2),R(3,3));
a = atan2(R(2,3)/sin(b),R(1,3)/sin(b));
g = atan2(R(3,2)/sin(b),-R(3,1)/sin(b));
end