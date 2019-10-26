function g = snake_fk_cal(theta)
% the forward kinematics of reUsnake 
% g is a list of transformations
% g0 is the transformation of the link 1 ( 

%   the joint and link definition of reUsnake
%   ===>    --o--   |   --o--|--o--|--o--|--o--|--o--|8
% tether    joint1  link1 .... 

% TODO: consider passing these parameters from outside of the function
L = 0.0639;
d = 0.0508;

% make input vector Nx1
[N,M] = size(theta);
if N < M
    theta = theta';
end
[N,M] = size(theta);

g_ab = zeros(4,4,N);
g = zeros(4,4,N);
w = zeros(3,N);
q = zeros(3,N);
se_g = zeros(4,4,N);

R = [1 0 0; 0 0 -1; 0 1 0];  % helper matrix rotate 90 positive around x

for i=1:N
    g_ab(:,:,i) = [eye(3) [(i-1)*L;0;0]; [0 0 0 1]];
    w(:,i) = R^(i-1)*[0;0;1];
    q(:,i) = [(i-1)*L;0;0];
 
    se_g(:,:,i) = MatrixExp6(VecTose3([w(:,i);-cross(w(:,i),q(:,i))]*theta(i)));
end

for i=1:N
    g(:,:,i) = g_ab(:,:,i);
    for j=i:-1:1
        g(:,:,i) = se_g(:,:,j)*g(:,:,i);
    end
end


end