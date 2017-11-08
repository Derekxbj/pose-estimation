clear all;
close all;
% The main progream
while(true)
tic;
try
    I = imread('image.jpg');
    imshow(I,[]);
    hold on;
    
    % The points in the model's coordinate system (cm)
    P_M=[  -5     5     5    -5    ; 
           -5    -5     5     5    ; 
            0     0     0     0     ;
            1     1     1     1     ];  
    %       1     2     3     4  

    % Define camera parameters
    f = 504; % focal length in pixels 
    cx = 320;
    cy = 240;

    %intrinsic parameter matrix
    K=[f 0 cx;
       0 f cy;
       0 0 1]; 

    % Make an initial guess of the pose [ax ay az tx ty tz]
    x0 = [0; 0; 0; 0; 0; 23];
    
    position = image_position(I);
    
    % Using convexhull to label the order of points
    x_position = position(:,1);
    y_position = position(:,2);
    DT = delaunayTriangulation(x_position,y_position);
    k = convexHull(DT);

    for i = 1:4
        corners(i,:) = DT.Points(k(i), :);
    end

    % The actual position on the image
    y = [
        corners(1,1); % point 1, col (x)
        corners(1,2); % point 1, row (y)
        corners(2,1); % point 2, col (x)
        corners(2,2); % point 2, row (y)
        corners(3,1); % point 3, col (x)
        corners(3,2); % point 3, row (y)
        corners(4,1); % point 4, col (x)
        corners(4,2) ]; % point 4, row (y)

    % Using least squares pose estimation
    [dy, x] = leastSquareFit(x0, P_M, K, y);

    % Draw coordinate axes onto the image. Scale the length of the axes
    % according to the size of the model, so that the axes are visible.

    W = max(P_M,[],2) - min(P_M,[],2); % Size of model in X,Y,Z
    W = norm(W); % Length of the diagonal of the bounding box

    u0 = fProject(x, [0;0;0;1], K); % origin
    uX = fProject(x, [W;0;0;1], K); % unit X vector
    uY = fProject(x, [0;W;0;1], K); % unit Y vector
    uZ = fProject(x, [0;0;W;1], K); % unit Z vector

    line([u0(1) uX(1)], [u0(2) uX(2)], 'Color', 'r', 'LineWidth', 3);
    line([u0(1) uY(1)], [u0(2) uY(2)], 'Color', 'g', 'LineWidth', 3);
    line([u0(1) uZ(1)], [u0(2) uZ(2)], 'Color', 'b', 'LineWidth', 3);

%     txt1 = 'origin';
%     text(280,240,txt1, 'color', 'y','FontSize',18);

    line([320,320],[240,480], 'color', 'g' ,'LineWidth',3);
    txt2 = 'Y-axis';
    text(320,360,txt2, 'color', 'y');

    line([320,640],[240,240], 'color', 'r' ,'LineWidth',3);
    txt3 = 'X-axis';
    text(480,230,txt3, 'color', 'y');

    % Also print the pose onto the image.
    text(0,450,sprintf('ax=%.2f ay=%.2f az=%.2f tx=%.1f ty=%.1f tz=%.1f', ...
     rad2deg(x(1)), rad2deg(x(2)), rad2deg(x(3)), x(4), x(5), x(6)), ...
     'BackgroundColor', 'w', 'FontSize', 15);
    pause(1);
    
catch
    disp("Can not implement pose estimation!!");
    continue;
end
toc;
break;
end
