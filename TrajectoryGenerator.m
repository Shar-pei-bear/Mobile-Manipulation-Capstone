function [traj_ref] = TrajectoryGenerator(Tse_initial,Tsc_initial,Tsc_final,Tce_grasp,Tce_standoff,k)
%Generate the reference trajectory for the end-effector frame {e}. 
%   Input:
%   The initial configuration of the end-effector in the reference trajectory: Tse,initial.
%   The cube's initial configuration: Tsc,initial.
%   The cube's desired final configuration: Tsc,final.
%   The end-effector's configuration relative to the cube when it is grasping the cube: Tce,grasp.
%   The end-effector's standoff configuration above the cube, before and after grasping, relative to the cube: Tce,standoff. 
%   The number of trajectory reference configurations per 0.01 seconds: k.
%   Outputs:
%   A representation of the N configurations of the end-effector along the entire concatenated eight-segment reference trajectory.
Tse_standoff  = Tsc_initial*Tce_standoff;
Tse_standoff2 = Tsc_final  *Tce_standoff;

Tse_grasp  = Tsc_initial*Tce_grasp;
Tse_grasp2 = Tsc_final  *Tce_grasp;

T_initial2standoff = TransInv(Tse_initial)*Tse_standoff;
T_standoff2grasp   = TransInv(Tce_standoff)*Tce_grasp;
T_inital2final     = TransInv(Tsc_initial)*Tsc_final;

[R_initial2standoff, p_initial2standoff] = TransToRp(T_initial2standoff);
so3mat_initial2standoff = MatrixLog3(R_initial2standoff);
w_initial2standoff = so3ToVec(so3mat_initial2standoff);

[R_standoff2grasp, p_standoff2grasp] = TransToRp(T_standoff2grasp);
so3mat_standoff2grasp = MatrixLog3(R_standoff2grasp);
w_standoff2grasp = so3ToVec(so3mat_standoff2grasp);

[R_inital2final, p_inital2final] = TransToRp(T_inital2final);
so3mat_inital2final = MatrixLog3(R_inital2final);
w_inital2final = so3ToVec(so3mat_inital2final);

[~, theta_initial2standoff] = AxisAng3(w_initial2standoff);
[~, theta_standoff2grasp] = AxisAng3(w_standoff2grasp);
[~, theta_inital2final] = AxisAng3(w_inital2final);

d_initial2standoff = norm(p_initial2standoff);
d_standoff2grasp = norm(p_standoff2grasp);
d_inital2final = norm(p_inital2final);

V = 1;
V = V/4;
W = pi/2/sqrt(2);
W = W/4;

t_initial2standoff = max(d_initial2standoff/V,theta_initial2standoff/W);
t_standoff2grasp = max(d_standoff2grasp/V,theta_standoff2grasp/W);
t_inital2final = max(d_inital2final/V,theta_inital2final/W);

t_initial2standoff = ceil(t_initial2standoff/0.01)*0.01;
t_standoff2grasp = ceil(t_standoff2grasp/0.01)*0.01;
t_inital2final = ceil(t_inital2final/0.01)*0.01;
t_closing = ceil(0.625/0.01)*0.01;

N_initial2standoff = k*ceil(t_initial2standoff/0.01) + 1;
N_standoff2grasp = k*ceil(t_standoff2grasp/0.01) + 1;
N_inital2final = k*ceil(t_inital2final/0.01) + 1;
N_closing = k*ceil(t_closing/0.01);

method = 5;
traj1 = ScrewTrajectory(Tse_initial, Tse_standoff, t_initial2standoff, N_initial2standoff, method);
traj2 = ScrewTrajectory(Tse_standoff, Tse_grasp, t_standoff2grasp,N_standoff2grasp, method);
traj3 = repmat({Tse_grasp},1,N_closing);
traj4 = ScrewTrajectory(Tse_grasp, Tse_standoff, t_standoff2grasp,N_standoff2grasp, method);
traj5 = ScrewTrajectory(Tse_standoff, Tse_standoff2, t_inital2final,N_inital2final, method);
traj6 = ScrewTrajectory(Tse_standoff2, Tse_grasp2, t_standoff2grasp,N_standoff2grasp, method);
traj7 = repmat({Tse_grasp2},1,N_closing);
traj8 = ScrewTrajectory(Tse_grasp2, Tse_standoff2, t_standoff2grasp,N_standoff2grasp, method);


traj = [traj1,traj2(1,2:end),traj3,traj4,traj5(1,2:end),traj6(1,2:end),traj7,traj8];
traj_ref = zeros(size(traj,2),13);
closing_point = size(traj1,2) + size(traj2,2) - 1;
opening_point = size(traj ,2) - size(traj8,2) - size(traj7,2);
for i=1:size(traj,2)
    T_current = traj{1,i};
    [R_current, p_current] = TransToRp(T_current);
     R_current = R_current';
     traj_ref(i,1:9) = R_current(:);
     traj_ref(i,10:12) = p_current;
    if i > closing_point && i <= opening_point
        traj_ref(i,13) = 1;
    else
        traj_ref(i,13) = 0;
    end
end
end

