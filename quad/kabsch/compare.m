p = [0;0;0];
o = [1,0,0;0,1,0;0,0,1];
pos = [p'];


% figure; hold on;
for i = 0:19
    load(['e-', num2str(i),'.mat'])

    [Rm, tm, a] = Kabsch(P', Q');
    
    o = o*Rm;
    
    p = p + o*tm;
    
    pos = [pos; p'];

end

figure

% a = -pi/4;
% Ry = [cos(a), 0 , sin(a); 0,1,0; -sin(a), 0, cos(a)];
% 
% pos = [-pos(:,3), pos(:,1), pos(:,2)];
% pos = (Ry*pos')';
scatter3(pos(:,1),pos(:,2),pos(:,3))