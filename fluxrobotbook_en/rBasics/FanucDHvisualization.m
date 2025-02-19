% 6-Axis Robot Visualization Using DH Parameters
%clc; clear;

% Joint coordinates visualized
J = deg2rad([0 0 0 10 20 50]); TCP=[-168 0];
J = deg2rad([0 0 0 0 0 0]); TCP=[-168 0];

J=[J 0]; %add for pointer
J(3)=J(3)+J(2); %Fanuc speciality
userframe=eulerAnglesToRotation3d(0,0,0,'XYZ');
% Define DH Parameters [theta, d, ar, alpha]

DH = [
    0, 0, 0, -pi/2;     % TO Joint 2
    -pi/2, 0, 840, pi;   % Joint 3
    pi, 0, -215, pi/2;    % Joint 4
    0, -965, 0, -pi/2;   % Joint 5
    0, 0, 0, pi/2;     % Joint 6
    -pi/2, -90, 0, pi ;       % 
    pi, -TCP(1), -TCP(2) ,pi/2       % TCP
];

T = eye(4); %world coordinates
num_joints = size(DH, 1);
pos = zeros(num_joints+1, 3);% Store positions of each join

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
 userframe=userframe(1:3,1:3); pos = userframe*pos'; pos=pos';%USER frame only rotatws for drawing

for ii = 1:num_joints-1    plot3(pos(ii:ii+1, 1), pos(ii:ii+1, 2), pos(ii:ii+1, 3), 'LineWidth', 3, 'Color', 'b');end % Blue lines for links
scatter3(pos(:, 1), pos(:, 2), pos(:, 3), 50, 'r', 'filled'); % Red dots for joints
for ii = 1:num_joints-1;    text(pos(ii, 1), pos(ii, 2), pos(ii, 3), sprintf('J%d', ii), 'FontSize', 10, 'Color', 'k'); end
cord=[1000, -1000,-700; 1000,-1000,700;-1000,-1000,700;-1000, -1000,-700];plot3(cord(:,1),cord(:,2),cord(:,3),'k'); %plot machine in black
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)'); title('6-Axis Robot Kinematics Visualization');
view([-135 30]);axis([-1.3 1.7 -1.5 1.5 -0.5 2]*1e3);
clear a_j Al  a_i alpha_i   cord    cord_ d_i i num_joints


cord=[50,0,0;0,0,0;0,50,0;0,0,0;0,0,50]*6;    plot3(cord(:,1),cord(:,2),cord(:,3),'b','Linewidth',1);text(cord(5,1),cord(5,2),cord(5,3),'z'); %Coord system
analysisDH=3; if analysisDH>0 %Show DH coord system of the Joint AFTER full DH transformaton
    T_ = eulerAnglesToRotation3d(0, 0 ,0,'XYZ')*TDH{analysisDH};
    T_(1:3,4)=TDH{analysisDH}(1:3,4);  %move the Matrix back to the TCP
    cord_ = transformPoint3d(cord, T_); plot3(cord_(:,1),cord_(:,2),cord_(:,3),'c','Linewidth',1); %Reference
    text(cord_(1,1),cord_(1,2),cord_(1,3),'x');text(cord_(3,1),cord_(3,2),cord_(3,3),'y');text(cord_(5,1),cord_(5,2),cord_(5,3),'z');
end;

titstr=sprintf('J1:%.2f J2:%.2f J3:%.2f J4:%.2f J5:%.2f J6:%.2f (J(3)=J(3)+J(2))\n',J(1:6)/pi*180); title(titstr);
[w] = rotation3dToEulerAngles(T,'ZYX'); 
titstr=[titstr,sprintf('TCP: %.2f %.2f %.2f , w:%.2f p:%.2f r:%.2f\n',pos(end,:),w(3),w(2),w(1))];title(titstr); %last rotation

%% Compute inverse kinematics (Without user frames
TPos=pos(end,:)';   % TCP pos
TOri=T(1:3,1:3);    % TCP ori
WP=TPos-TOri*[TCP(2),-TCP(1)-DH(6,2), 0 ]'; % Wrist point

% TCP visualization for invers
T_ = eulerAnglesToRotation3d(w,'ZYX');
T_(1:3,4)=T(1:3,4);  %move the Matrix back to the TCP
cord_ = transformPoint3d(cord, T_); plot3(cord_(:,1),cord_(:,2),cord_(:,3),'c','Linewidth',1); %Reference
text(cord_(1,1),cord_(1,2),cord_(1,3),'x');text(cord_(3,1),cord_(3,2),cord_(3,3),'y');text(cord_(5,1),cord_(5,2),cord_(5,3),'z');
cord_ = transformPoint3d(cord, T); plot3(cord_(:,1),cord_(:,2),cord_(:,3),'b--','Linewidth',1); %Reference
text(cord_(1,1),cord_(1,2),cord_(1,3),'x');text(cord_(3,1),cord_(3,2),cord_(3,3),'y');text(cord_(5,1),cord_(5,2),cord_(5,3),'z');

% Joint 1 (theta1): Base rotation
J1 = atan2(WP(2), WP(1))/pi*180;

% Joint 2 (theta2) and Joint 3 (theta3): Arm angles
ra=sqrt(DH(3,3)^2+DH(4,2)^2);
r = sqrt(WP(1)^2 + WP(2)^2+WP(3)^2);
r2d = sqrt(WP(1)^2 + WP(2)^2);
J2=acos((ra^2-DH(2,3)^2-r^2)/(-2*DH(2,3)*r));   %triangle angle
J2=atan(WP(3)/r2d)+J2;
J2=90- J2/pi*180;
J3=acos((r^2-ra^2-DH(2,3)^2)/(-2*ra*DH(2,3)));   %triangle
J3=J3-(atan(DH(3,3)/DH(4,2)));                  %d1 piece
J3=J3/pi*180 -90;                              %Fanuc nomemclature -90=zero is rect
J3=J3-J2;

% Joint 4 to Joint 6: Orientation
TW2=createRotationOz(-J1/180*pi)*T;
TW2 = createRotationOy(J3/180*pi)*TW2;  %Fanuc nomenclature J2 eliminated!
TW2=TW2*eulerAnglesToRotation3d(0, 0,90,'XYZ'); %rotate in Target
TW2(1:3,4)=0;

cord_ = transformPoint3d(cord, TW2); plot3(cord_(:,1),cord_(:,2),cord_(:,3),'r','Linewidth',1); %Reference
text(cord_(1,1),cord_(1,2),cord_(1,3),'x');text(cord_(3,1),cord_(3,2),cord_(3,3),'y');text(cord_(5,1),cord_(5,2),cord_(5,3),'z');
[w] = rotation3dToEulerAngles(TW2,'XYX'); 
titstr=[titstr,sprintf('inv: J1:%.2f J2:%.2f J3:%.2f J4:%.2f J5:%.2f J6:%.2f\n',J1,J2,J3,-w(1),w(2),-w(3))];title(titstr); %last rotation
