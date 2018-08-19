function [configs,Xerrs] = MobileManipulation(Tse_initial,Tsc_initial,Tsc_final,config_initial,Kp,Ki)
%UNTITLED 此处显示有关此函数的摘要
%   input:
%   the initial resting configuration of the cube object
%   the desired final resting configuration of the cube object
%   the actual initial configuration of the youBot
%   the reference initial configuration of the youBot 
%   gains for feedback controller
%   output:
%   the configurations of the robot as a function of time
%   the 6-vector end-effector error as a function of time

k = 1;
deltat = 0.01;
Tce_grasp = [ 0,0,1,0
              0,1,0,0
             -1,0,0,0
              0,0,0,1];
Tce_standoff = [ 0,0,1, 0
                 0,1,0, 0
                -1,0,0, 0.2
                 0,0,0, 1];
traj_ref = TrajectoryGenerator(Tse_initial,Tsc_initial,Tsc_final,Tce_grasp,Tce_standoff,k);
csvwrite('C:\Program Files\V-REP3\V-REP_PRO_EDU\scenes\customscenes\trajref.csv',traj_ref);
w1 = [0, 0,1]; 
w2 = [0,-1,0];
w3 = [0,-1,0];
w4 = [0,-1,0];
w5 = [0, 0,1];

v1 = [0      ,0.033,0];
v2 = [-0.5076,0    ,0];
v3 = [-0.3526,0    ,0];
v4 = [-0.2176,0    ,0];
v5 = [0      ,0    ,0];

B1 =[w1,v1]';
B2 =[w2,v2]';
B3 =[w3,v3]';
B4 =[w4,v4]';
B5 =[w5,v5]';

Blist = [B1,B2,B3,B4,B5];
Tb0 = [1,0,0,0.1662;0,1,0,0;0,0,1,0.0026;0,0,0,1];
M0e = [1,0,0,0.033;0,1,0,0;0,0,1,0.6546;0,0,0,1];
Ve_i = zeros(6,1);

l = 0.47/2;
w = 0.3 /2;
r = 0.0475;
F = r/4*[-1/(l+w),1/(l+w),1/(l+w),-1/(l+w)
                1,      1,      1,       1
               -1,      1,     -1,       1];
F6 = zeros(6,4);
F6(3:5,:) = F;

max_omg =  10;
min_omg = -10;
configs = zeros(size(traj_ref,1),12);
Xerrs = zeros(size(traj_ref,1),6);
Xerrs(1,:) = [0,0,0,0,0,0];
configs(1,:) = config_initial;
config_cur   = config_initial';
for i = 1:(size(traj_ref,1)-1)
    thetalist = config_cur(4:8);
    Tsb = [cos(config_cur(1)),-sin(config_cur(1)),0,config_cur(2);sin(config_cur(1)),cos(config_cur(1)),0,config_cur(3);0,0,1,0.0963;0,0,0,1];
    T0e = FKinBody(M0e,Blist,thetalist);
    X = Tsb*Tb0*T0e;
    pd = traj_ref(i,10:12)';
    Rd = traj_ref(i,1:9);
    Rd = reshape(Rd,[3,3])';
    Xd = RpToTrans(Rd,pd);
    
    pd_next = traj_ref(i+1,10:12)';
    Rd_next = traj_ref(i+1,1:9);
    Rd_next = reshape(Rd_next,[3,3])';
    Xd_next = RpToTrans(Rd_next,pd_next);
    [Ve,Ve_i,Xerr] = FeedbackControl(Xd,Xd_next,X,Kp,Ki,deltat/k,Ve_i);
    Jarm = JacobianBody(Blist, thetalist);
    Jbase = Adjoint(TransInv(T0e)*TransInv(Tb0))*F6;
    Je = [Jbase,Jarm];
    u = pinv(Je)*Ve;
    controls = [u(5:end);u(1:4)];
    config_cur = NextState(config_cur,controls,deltat/k,max_omg, min_omg);
    configs(i+1,:) = config_cur;
    Xerrs(i+1,:) = Xerr;
end
configs = [configs,traj_ref(:,end)];
end

