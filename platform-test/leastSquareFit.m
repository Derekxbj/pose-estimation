function [dy, x] = leastSquareFit(x, P_M, K, y0)
% Using least square to minimize the errors

for i=1:10
    % Get predicted image points
    y = fProject(x, P_M, K);
    
%     for i=1:2:length(y)
%         rectangle('Position', [y(i)-8 y(i+1)-8 16 16], ...
%             'FaceColor', 'r');
%     end

    % Estimate Jacobian
    e = 0.00001; % a tiny number
    J(:,1) = ( fProject(x+[e;0;0;0;0;0],P_M,K) - y )/e;
    J(:,2) = ( fProject(x+[0;e;0;0;0;0],P_M,K) - y )/e;
    J(:,3) = ( fProject(x+[0;0;e;0;0;0],P_M,K) - y )/e;
    J(:,4) = ( fProject(x+[0;0;0;e;0;0],P_M,K) - y )/e;
    J(:,5) = ( fProject(x+[0;0;0;0;e;0],P_M,K) - y )/e;
    J(:,6) = ( fProject(x+[0;0;0;0;0;e],P_M,K) - y )/e;


    % Error is observed image points - predicted image points
    dy = y0 - y;
%     fprintf('Residual error: %f\n', norm(dy));

    % a system of linear equations dy = J dx
    % Solve for dx using the pseudo inverse
    dx = pinv(J) * dy;
%     dx = inv(J'*J)*J'*dy;

    % Stop if parameters are no longer changing
    if abs( norm(dx)/norm(x) ) < 1e-6 
        break;
    end
        
    x = x + dx; % Update pose estimate
%     pause(1);
end

    for i=1:2:length(y)
        rectangle('Position', [y(i)-8 y(i+1)-8 16 16], ...
            'FaceColor', 'r');
    end

dy = y0 - y;
fprintf('Residual error: %f\n', norm(dy));
end