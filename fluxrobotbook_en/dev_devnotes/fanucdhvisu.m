% 6-Axis Robot Visualization Using DH Parameters
clc; clear;
    analysisDH=5;
% Joint coordinates visualized
J = deg2rad([20 0 0 0 20 0]); TCP=[-168 0];
J(3)=J(3)+J(2); %Fanuc speciality

% Define DH Parameters [theta, d, ar, alpha]

DH = [
    0, 0, 0, -pi/2;     % TO Joint 2
    -pi/2, 0, 840, pi;   % Joint 3
    pi, 0, -215, pi/2;    % Joint 4
    0, -890, 0, -pi/2;   % Joint 5
    0, 0, 0, pi/2;     % Joint 6
    0, -165, 0, 0        % 
    0, TCP(1), TCP(2) , 0        % TCP
];

% Define joint angles (theta) in radians (example configuration)

J=[J 0]; %add for pointer

% Number of joints
num_joints = size(DH, 1);

% Initialize transformation matrix
T = eye(4);

% Store positions of each joint
pos = zeros(num_joints+1, 3);

% Loop through each joint to compute forward kinematics
for ii = 1:num_joints
    % Extract DH parameters for the current joint
    theta_i = J(ii) + DH(ii, 1); % Joint angle + offset
    d_i = DH(ii, 2); % Link offset
    a_i = DH(ii, 3); % Link length
    alpha_i = DH(ii, 4); % Link twist
    
    % Compute the transformation matrix for the current joint
    A_i = [
        cos(theta_i), -sin(theta_i)*cos(alpha_i),  sin(theta_i)*sin(alpha_i), a_i*cos(theta_i);
        sin(theta_i),  cos(theta_i)*cos(alpha_i), -cos(theta_i)*sin(alpha_i), a_i*sin(theta_i);
        0,             sin(alpha_i),              cos(alpha_i),              d_i;
        0,             0,                         0,                         1
    ];
    T = T * A_i; % Update the overall transformation matrix
    TDH{ii}=T;   % store partial rotation for diagnosis
    pos(ii+1, :) = T(1:3, 4)';
end

% Plot the robot in 3D
clf; hold on; grid on; axis equal;

for ii = 1:num_joints-1    plot3(pos(ii:ii+1, 1), pos(ii:ii+1, 2), pos(ii:ii+1, 3), 'LineWidth', 3, 'Color', 'b');end % Blue lines for links
scatter3(pos(:, 1), pos(:, 2), pos(:, 3), 50, 'r', 'filled'); % Red dots for joints
for ii = 1:num_joints-1;    text(pos(ii, 1), pos(ii, 2), pos(ii, 3), sprintf('J%d', ii), 'FontSize', 10, 'Color', 'k'); end
cord=[1000, -1000,-700; 1000,-1000,700;-1000,-1000,700;-1000, -1000,-700];plot3(cord(:,1),cord(:,2),cord(:,3),'k'); %plot machine in black
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)'); title('6-Axis Robot Kinematics Visualization');
view([-135 30]);axis([-1.5 1.5 -1.5 1.5 -0.1 2]*1e3);
clear a_j Al  a_i alpha_i   cord    cord_ d_i i num_joints

cord=[50,0,0;0,0,0;0,50,0;0,0,0;0,0,50]*6;% Coord system
if 0 %Diangosis coords; shows coordinate after Rotation of joont 
    plot3(cord(:,1),cord(:,2),cord(:,3),'b','Linewidth',1);text(cord(5,1),cord(5,2),cord(5,3),'z');
    T_ = eulerAnglesToRotation3d(0, 0 ,0,'XYZ')*TDH{analysisDH};
    T_(1:3,4)=TDH{analysisDH}(1:3,4);  %move the Matrix back to the TCP
    cord_ = transformPoint3d(cord, T_); plot3(cord_(:,1),cord_(:,2),cord_(:,3),'c','Linewidth',1); %Reference
    text(cord_(1,1),cord_(1,2),cord_(1,3),'x');text(cord_(3,1),cord_(3,2),cord_(3,3),'y');text(cord_(5,1),cord_(5,2),cord_(5,3),'z');
end;

[w] = rotation3dToEulerAngles(T,'ZYX');

T_ = eulerAnglesToRotation3d(w,'ZYX');
T_(1:3,4)=T(1:3,4);  %move the Matrix back to the TCP
cord_ = transformPoint3d(cord, T_); plot3(cord_(:,1),cord_(:,2),cord_(:,3),'g','Linewidth',1); %Reference
text(cord_(1,1),cord_(1,2),cord_(1,3),'x');text(cord_(3,1),cord_(3,2),cord_(3,3),'y');text(cord_(5,1),cord_(5,2),cord_(5,3),'z');

[w] = rotation3dToEulerAngles(T,'ZYX');
fprintf('TCP: %.2f %.2f %.2f , ori: %.2f %.2f %.2f\n',pos(end,:),w(3)+180,w(2),w(1)); %last rotation

%% inverse trafo
TPos=pos(end,:)';% Desired end-effector pos
WP=[TPos; 0]+T*[0 TCP(2), -TCP(1)+165 0]';

% Compute inverse kinematics
% Initialize joint angles (theta1 to theta6)
theta = zeros(1, 6);

% Joint 1 (theta1): Base rotation
J1 = atan2(TPos(2), TPos(1))/pi*180;

% Joint 2 (theta2) and Joint 3 (theta3): Arm angles
ra=sqrt(DH(3,3)^2+DH(4,2)^2);
r = sqrt(WP(1)^2 + WP(2)^2+WP(3)^2);
J3=acos((r^2-ra^2-DH(2,3)^2)/(-2*ra*DH(2,3)));   %triangle
J3=J3-(atan(DH(3,3)/DH(4,2)));                  %d1 piece
J3=J3/pi*180  -90;                              %-90=zero is rect
% Law of cosines for intermediate angle beta
r2d = sqrt(WP(1)^2 + WP(2)^2);
J2=acos((ra^2-DH(2,3)^2-r^2)/(-2*DH(2,3)*r));   %triangle angle
J2=atan(WP(3)/r2d)+J2;
J2=J2/pi*180 -90;

% Joint 4 to Joint 6: Orientation
TW = eulerAnglesToRotation3d(J1,-J2-J3,0,'ZYX');  %coordinates from base
TW = TW*eulerAnglesToRotation3d(180,90,0,'ZYX');  %turn into the fanuc frame
[w] = rotation3dToEulerAngles(T/TW,'ZYZ');        %Calculate needed rotation with ZYZ = J6,J5,J4

TW(1:3,4)=TDH{6}(1:3,4);  %move the Matrix back to the TCP
cord_ = transformPoint3d(cord, TW);
plot3(cord_(:,1),cord_(:,2),cord_(:,3),'c','Linewidth',1); %Reference
text(cord_(1,1),cord_(1,2),cord_(1,3),'x');text(cord_(3,1),cord_(3,2),cord_(3,3),'y');text(cord_(5,1),cord_(5,2),cord_(5,3),'z');
fprintf('inverse calculated axis:\n J1:%.2f J2:%.2f J3:%.2f J4:%.2f J5:%.2f J6:%.2f\n',J1,J2,J3,w);  
