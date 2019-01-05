function M = eul2rotm(eul)
a = eul(1);
b = eul(2);
g = eul(3);


s1 = sin(a);
s2 = sin(b);
s3 = sin(g);
c1 = cos(a);
c2 = cos(b);
c3 = cos(g);

%ZYZ
M = [c1*c2*c3-s1*s3, -c3*s1-c1*c2*s3, c1*s2;
    c1*s3+c2*c3*s1, c1*c3-c2*s1*s3, s1*s2;
    -c3*s2, s2*s3, c2];

% Rz0 = [cos(a), -sin(a), 0; sin(a), cos(a), 0; 0, 0, 1];
% Ry = [cos(b), 0, sin(b); 0,1,0; -sin(b), 0, cos(b)];
% Rzn = [cos(g), -sin(g), 0; sin(g), cos(g), 0; 0, 0, 1];


% u = -sin(a);
% v = cos(a);
% w = 0;
% t = b;
% Ry = [u^2+(1-u^2)*cos(t), u*v*(1-cos(t))-w*sin(t), u*w*(1-cos(t))+v*sin(t);
%     u*v*(1-cos(t))+w*sin(t), v^2+(1-v^2)*cos(t), v*w*(1-cos(t))-u*sin(t); 
%     u*w*(1-cos(t))-v*sin(t), v*w*(1-cos(t))+u*sin(t), w^2+(1-w^2)*cos(t)];    
% 
% u = cos(a)*sin(b);
% v = sin(a)*sin(b);
% w = cos(b);
% t = g;
% Rzn = [u^2+(1-u^2)*cos(t), u*v*(1-cos(t))-w*sin(t), u*w*(1-cos(t))+v*sin(t);
%     u*v*(1-cos(t))+w*sin(t), v^2+(1-v^2)*cos(t), v*w*(1-cos(t))-u*sin(t); 
%     u*w*(1-cos(t))-v*sin(t), v*w*(1-cos(t))+u*sin(t), w^2+(1-w^2)*cos(t)];   
% 
% 
% M = [Rz0,Ry,Rzn];
% M = Rzn*Ry*Rz0;

% u = -sin(a);
% v = cos(a);
% w = 0;
% 
% Ry = [(u^2+(v^2+w^2)*cos(b))/(u^2+v^2+w^2), ...
%     (u*v*(1-cos(b))-w*sqrt(u^2+v^2+w^2)*sin(b))/(u^2+v^2+w^2), ...
%     (u*w*(1-cos(b))+v*sqrt(u^2+v^2+w^2)*sin(b))/(u^2+v^2+w^2); ...
%     (u*v*(1-cos(b))+w*sqrt(u^2+v^2+w^2)*sin(b))/(u^2+v^2+w^2), ...
%     (v^2+(u^2+w^2)*cos(b))/(u^2+v^2+w^2), ...
%     (v*w*(1-cos(b))-u*sqrt(u^2+v^2+w^2)*sin(b))/(u^2+v^2+w^2); ...
%     (u*w*(1-cos(b))-v*sqrt(u^2+v^2+w^2)*sin(b))/(u^2+v^2+w^2), ...
%     (v*w*(1-cos(b))+u*sqrt(u^2+v^2+w^2)*sin(b))/(u^2+v^2+w^2), ...
%     (w^2+(u^2+v^2)*cos(b))/(u^2+v^2+w^2)];
% 
% u = cos(a)*sin(b);
% v = sin(a)*sin(b);
% w = cos(b);
% 
% Rzn = [(u^2+(v^2+w^2)*cos(g))/(u^2+v^2+w^2), ...
%     (u*v*(1-cos(g))-w*sqrt(u^2+v^2+w^2)*sin(g))/(u^2+v^2+w^2), ...
%     (u*w*(1-cos(g))+v*sqrt(u^2+v^2+w^2)*sin(g))/(u^2+v^2+w^2); ...
%     (u*v*(1-cos(g))+w*sqrt(u^2+v^2+w^2)*sin(g))/(u^2+v^2+w^2), ...
%     (v^2+(u^2+w^2)*cos(g))/(u^2+v^2+w^2), ...
%     (v*w*(1-cos(g))-u*sqrt(u^2+v^2+w^2)*sin(g))/(u^2+v^2+w^2); ...
%     (u*w*(1-cos(g))-v*sqrt(u^2+v^2+w^2)*sin(g))/(u^2+v^2+w^2), ...
%     (v*w*(1-cos(g))+u*sqrt(u^2+v^2+w^2)*sin(g))/(u^2+v^2+w^2), ...
%     (w^2+(u^2+v^2)*cos(g))/(u^2+v^2+w^2)];

% Rzn = [(u^2+w^2*cos(b))/(u^2+w^2),(-w*sqrt(u^2+w^2)*sin(b))/(u^2+w^2), ...
%     (u*w*(1-cos(b)))/(u^2+w^2); ...
%     (w*sqrt(u^2+w^2)*sin(b))/(u^2+w^2), ((u^2+w^2)*cos(b))/(u^2+w^2), ...
%     (-u*sqrt(u^2+w^2)*sin(b))/(u^2+w^2); ...
%     (u*w*(1-cos(b)))/(u^2+w^2), (u*sqrt(u^2+w^2)*sin(b))/(u^2+w^2), ...
%     (w^2+u^2*cos(b))/(u^2+w^2)];
end