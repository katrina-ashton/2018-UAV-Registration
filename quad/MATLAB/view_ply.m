folder = '../../../data/quad3/pcs/';

listing = dir(folder);
fns = cell(1,length(listing)-2);
for k = 3:length(listing)
    fns(k-2) = {listing(k).name};
end


% step =4;
% fns = fns(1:step:end);

ptCloud = pcread(strcat(folder,fns{1}));
% pcshow(ptCloud);

a = pi/2+pi/4;%2+pi/20;
R1 = [1, 0, 0; ...
      0, cos(a), -sin(a); ...
      0, sin(a), cos(a);];

a = pi/2;
R2 = [cos(a), -sin(a),0; ...
      sin(a), cos(a),0; ...
      0,0,1;];

% t = [-0.93270379 -0.01715724  1.3447715 ];
t = [0,0,0];
R3 = [ 9.84073180e-01  1.77763695e-01 -2.10786504e-04; ...
 -1.77762148e-01  9.84058094e-01 -5.50339411e-03; ...
 -7.70877505e-04  5.45321241e-03  9.99984834e-01];  
  
a = [0;0;0;1];

R = R1*R2;
A = [R; t];
A = [A, a];

tform1 = affine3d(A);

% R = R2;
% A = [R; t];
% A = [A, a];
% 
% tform2 = affine3d(A);

for i=5%size(fns,2)
    disp(fns{i})
    ptCloud = pcread(strcat(folder,fns{i}));
    ptCloud = pctransform(ptCloud, tform1);
%     ptCloud = pctransform(ptCloud, tform2);
    figure;
    pcshow(ptCloud);
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
%     xlim([0 2])
%     ylim([-1 1])
%     zlim([-1.5 -0.5])
%     xlim([-1 1])
%     ylim([-1 1])
%     zlim([0 2])

end

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