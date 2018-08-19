function [config_nex] = NextState(config_cur,controls,deltat,max_omg, min_omg)
%a simulator for the kinematics of the youBot.
% Input:
% A 12-vector representing the current configuration of the robot (3 variables for the chassis configuration, 5 variables for the arm configuration, and 4 variables for the wheel angles).
% A 9-vector of controls indicating the arm joint speeds theta dot (5 variables) and the wheel speeds u (4 variables).
% A timestep ¦¤t.
% A positive real value indicating the maximum angular speed of the arm joints and the wheels. For example, if this value is 12.3, the angular speed of the wheels and arm joints is limited to the range [-12.3 radians/s, 12.3 radians/s]. Any speed in the 9-vector of controls that is outside this range will be set to the nearest boundary of the range. If you don't want speed limits, just use a very large number. If you prefer, your function can accept separate speed limits for the wheels and arm joints.
% Output: A 12-vector representing the configuration of the robot time ¦¤t later.
% The function NextState is based on a simple first-order Euler step, i.e.,
% new arm joint angles = (old arm joint angles) + (joint speeds) * ¦¤t
% new wheel angles = (old wheel angles) + (wheel speeds) * ¦¤t
% new chassis configuration is obtained from odometry.
%   
%
controls = max(min_omg, controls);
controls = min(max_omg, controls);

config_nex = config_cur;
config_nex(4:12) = wrapToPi(config_nex(4:12) + controls*deltat);

l = 0.47/2;
w = 0.3 /2;
r = 0.0475;
H_inv = r/4*[-1/(l+w),1/(l+w),1/(l+w),-1/(l+w)
                    1,      1,      1,       1
                   -1,      1,     -1,       1];
V_b = H_inv*controls(6:9)*deltat;
if V_b(1) == 0
    delta_qb = V_b';
else
    delta_qb(1) = V_b(1);
    delta_qb(2) = (V_b(2)*sin(V_b(1)) + V_b(3)*(cos(V_b(1))-1))/V_b(1);
    delta_qb(3) = (V_b(3)*sin(V_b(1)) + V_b(2)*(1-cos(V_b(1))))/V_b(1);
end
Tsb = [1,0,0;0,cos(config_cur(1)),-sin(config_cur(1));0,sin(config_cur(1)),cos(config_cur(1))];
delta_qs = Tsb*delta_qb';
config_nex(1:3) = config_nex(1:3) + delta_qs;
config_nex(1) = wrapToPi(config_nex(1));
end

