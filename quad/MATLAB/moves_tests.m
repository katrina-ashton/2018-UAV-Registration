folder = 'F:/pointclouds/move tests/slow/2/';

listing = dir(folder);
fns = cell(1,length(listing)-2);
for k = 3:length(listing)
    fns(k-2) = {listing(k).name};
end

ptCloud1 = pcread(strcat(folder,fns{length(listing)/2}));


folder = 'F:/pointclouds/move tests/fast/2/';

listing = dir(folder);
fns = cell(1,length(listing)-2);
for k = 3:length(listing)
    fns(k-2) = {listing(k).name};
end

% ptCloud2 = pcread(strcat(folder,fns{length(listing)/2}));
ptCloud2 = pcread(strcat(folder,fns{2}));

pcshowpair(ptCloud1,ptCloud2)

[tform, ptCloudReg, rmse] = pcregistericp(ptCloud1,ptCloud2);

rmse
%tform.T
%[a,b,g] = rotm2euler(tform.T)
sum(sum((tform.T-eye(4)).^2))