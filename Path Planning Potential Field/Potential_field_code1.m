clc;
clear ;
close all;

% Input map
RGB = imread('mapn.png');
figure('Position',[600 0 600 1000],'color','k');
image(RGB);
hold on;
axis off
axis image

% Convert to binary image matrix and inverse matrix values
I = rgb2gray(RGB);
binaryImage = im2bw(RGB, 0.3);
binaryImage = 1-binaryImage;
% Inital pose
xs = 540; ys = 750;
% Goal Pose
xg = 150; yg = 150;

% Attractive Potential
sz  = size(binaryImage);
for i = 1:sz(1)
    for j = 1:sz(2)
       rg = sqrt((i-yg)^2 +(j - xg)^2);
       U_att(i,j) = 0.5*rg^2;
    end
end
contour(U_att,40);

set(gcf, 'InvertHardCopy', 'off');

%  Matrix
pose = zeros(sz(1),sz(2));
pose(ys,xs) = 1;
pose(yg,xg) = 1;
spy(pose,'*r');
title({'Attractive Potential Field';...
   'Circles Center Corespondents to the Goal Position '},'color','w');
hold off;

% Repulsive Potential
Di = 35; % Distance of influence of the object
% Compute for every pixel the distance to the nearest obstacle(non zero
% elements which after inversion corespondent to the obstacles)
[D,IDX] = bwdist(binaryImage);
for i = 1:sz(1)
    for j = 1:sz(2)
        rd = D(i,j);
        if (rd <= Di)
            U_rep(i,j)  = 0.5*(rd - Di)^2;
        else
            U_rep(i,j)  = 0;
        end
    end
end
figure('Position',[600 0 600 1000],'color','k');
hold on;
colormap jet;
contourf(U_rep,5);
spy(pose,'*r');
axis off
axis image
set(gcf, 'InvertHardCopy', 'off');
title('Repulsive Potential Field','color','w');
hold off;

% Summary Potential field
k_s  = 2; % Scaling factor
U_sum = U_rep/max(U_rep(:))+k_s*U_att/max(U_att(:));
figure('Position',[600 0 600 1000],'color','k');
hold on;
contourf(U_sum,15);
spy(pose,'*r');
axis off
axis image
title('Summary Potential Field','color','w');
set(gcf, 'InvertHardCopy', 'off');
hold off;

% Potential Field Path Planning

figure('Position',[600 0 600 1000],'color','k');
hold on;
contourf(U_sum,15);
spy(pose,'*r');
axis off
axis image
title('Potential Field Path Planning,*Note the Potential Trap','color','w');
x = xs; y = ys;
last = U_sum(y-1,x-1);
while (x ~= xg)||(y ~= yg)
    dis =[ U_sum(y-1,x-1), U_sum(y-1,x),U_sum(y-1,x+1);
             U_sum(y,x-1), U_sum(y,x)  ,U_sum(y,x+1)  ;
           U_sum(y+1,x-1), U_sum(y+1,x),U_sum(y+1,x+1)];
     m = min(dis(:));
     [r,c] = find(dis == m);
   U_sum(y,x) = inf;
   y = y-2+r;
   x = x-2+c;
   pose(y,x) = 1;
  end
spy(pose,'.r');
set(gcf, 'InvertHardCopy', 'off');
hold off;