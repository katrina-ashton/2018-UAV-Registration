folder = '../../../data/quad3/pcs/';

listing = dir(folder);
fns = cell(1,length(listing)-2);
for k = 3:length(listing)
    fns(k-2) = {listing(k).name};
end


% step =4;
% fns = fns(700:step:2800);
% fns = fns(1:step:end);

ptCloud = pcread(strcat(folder,fns{1}));

a = pi/2+pi/4;
R1 = [1, 0, 0; ...
      0, cos(a), -sin(a); ...
      0, sin(a), cos(a);];

a = pi/2;
R2 = [cos(a), -sin(a),0; ...
      sin(a), cos(a),0; ...
      0,0,1;];

t = [0,0,0];
R3 = [ 9.84073180e-01  1.77763695e-01 -2.10786504e-04; ...
 -1.77762148e-01  9.84058094e-01 -5.50339411e-03; ...
 -7.70877505e-04  5.45321241e-03  9.99984834e-01];  
  
a = [0;0;0;1];

R = R1*R2;
A = [R; t];
A = [A, a];

tformc = affine3d(A);

R = eul2rotm([-5.45324106e-03, -7.70877581e-04,  1.78711966e-01]);
t = [-0.93270379, -0.01715724, 1.3447715];
B = [R; t];
B = [B, a];

tformq = affine3d(B);

Rs = [];
ts = [];

Rsr = [];
tsr = [];

% figure;
gridSize = 0.1;
tic;
skip = 5;
for i=1:skip:size(fns,2)-1
    pc1 = pcread(strcat(folder,fns{i}));
    pc1 = pcdownsample(pc1, 'gridAverage', gridSize);
    pc1 = pctransform(pc1, tformc);
%     pc1 = pctransform(pc1, tformq);
    
    pc2 = pcread(strcat(folder,fns{i+1}));
    pc2 = pcdownsample(pc2, 'gridAverage', gridSize);
    pc2 = pctransform(pc2, tformc);
%     pc2 = pctransform(pc2, tformq);
    
    tform = pcregistericp(pc1, pc2, 'Metric','pointToPlane','Extrapolate', true);


    pctransform(pc2,tform);
%     pcshow(pc1)
%     pcshow(pc2)
    
    Rt = tform.T(1:3,1:3);
    tt = tform.T(4,1:3);
    R = R*Rt';
    t = t - (R*tt')';
    Rs = cat(3,Rs,R);
    ts = [ts; t];
    Rsr = cat(3,Rsr,Rt');
    tsr = [tsr; -tt];
    
    B = [R; t];
    B = [B, a];
    
    tformq = affine3d(B);

end
tm = toc;
save(['tm-' num2str(skip) '.mat'],'tm')
save(['Rs-' num2str(skip) '.mat'],'Rs')
save(['ts-' num2str(skip) '.mat'],'ts')
save(['Rsr-' num2str(skip) '.mat'],'Rsr')
save(['tsr-' num2str(skip) '.mat'],'tsr')

% for i=5%size(fns,2)
%     disp(fns{i})
%     ptCloud = pcread(strcat(folder,fns{i}));
%     ptCloud = pctransform(ptCloud, tformc);
% %     ptCloud = pctransform(ptCloud, tform2);
%     figure;
%     pcshow(ptCloud);
%     xlabel('X')
%     ylabel('Y')
%     zlabel('Z')
%     xlim([0 2])
%     ylim([-1 1])
%     zlim([-1.5 -0.5])
% %     xlim([-1 1])
% %     ylim([-1 1])
% %     zlim([0 2])
% 
% end



% ptCloudRef = pcread(strcat(folder,fns{1}));
% ptCloudCurrent = pcread(strcat(folder,fns{2}));
% 
% gridSize = 0.1;
% fixed = pcdownsample(ptCloudRef, 'gridAverage', gridSize);
% moving = pcdownsample(ptCloudCurrent, 'gridAverage', gridSize);
% 
% tform2 = pcregistericp(moving, fixed, 'Metric','pointToPlane','Extrapolate', true);
% ptCloudAligned = pctransform(ptCloudCurrent,tform2);
% 
% mergeSize = 0.015;
% ptCloudScene = pcmerge(ptCloudRef, ptCloudAligned, mergeSize);
% 
% accumTform = tform2;
% 
% figure
% hAxes = pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down');
% title('Updated world scene')
% % Set the axes property for faster rendering
% hAxes.CameraViewAngleMode = 'auto';
% hScatter = hAxes.Children;
% 
% hScatter.XData = ptCloudScene.Location(:,1);
%     hScatter.YData = ptCloudScene.Location(:,2);
%     hScatter.ZData = ptCloudScene.Location(:,3);
%     hScatter.CData = ptCloudScene.Color;
%     drawnow('limitrate')
%     pause(15)
% 
% for i = 3:length(fns)
%     ptCloudCurrent = pcread(strcat(folder,fns{i}));
% 
%     % Use previous moving point cloud as reference.
%     fixed = moving;
%     moving = pcdownsample(ptCloudCurrent, 'gridAverage', gridSize);
% 
%     % Apply ICP registration.
%     tform = pcregistericp(moving, fixed, 'Metric','pointToPlane','Extrapolate', true);
% 
%     % Transform the current point cloud to the reference coordinate system
%     % defined by the first point cloud.
%     accumTform = affine3d(tform.T * accumTform.T);
%     ptCloudAligned = pctransform(ptCloudCurrent, accumTform);
% 
%     % Update the world scene.
%     ptCloudScene = pcmerge(ptCloudScene, ptCloudAligned, mergeSize);
% 
%     % Visualize the world scene.
%     hScatter.XData = ptCloudScene.Location(:,1);
%     hScatter.YData = ptCloudScene.Location(:,2);
%     hScatter.ZData = ptCloudScene.Location(:,3);
%     hScatter.CData = ptCloudScene.Color;
%     drawnow('limitrate')
%     pause(1)
% end


% Visualize 
% figure
% pcshow(ptCloudScene)
% title('Initial world scene')
% xlabel('X (m)')
% ylabel('Y (m)')
% zlabel('Z (m)')
% drawnow

% tform1.T
% 
% tform2.T