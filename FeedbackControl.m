function [Ve,Ve_i,Xerr] = FeedbackControl(Xd,Xd_next,X,Kp,Ki,deltat,Ve_i)
%Calculate the kinematic task-space feedforward plus feedback control law
%   Input:
%   The current end-effector reference configuration Xd.
%   The end-effector reference configuration at the next timestep in the reference trajectory, Xd,next, at a time ¦¤t later.
%   The current actual end-effector configuration X (also written Tse).
%   The PI gain matrices Kp and Ki.
%   The timestep ¦¤t between reference trajectory configurations.
%   Output:
%   The commanded end-effector twist \mathcal{V} expressed in the end-effector frame {e}.
AdT = Adjoint(TransInv(X)*Xd);
Vd = se3ToVec(MatrixLog6(TransInv(Xd)*Xd_next))/deltat;
Xerr = se3ToVec(MatrixLog6(TransInv(X)*Xd));
Ve_i = Ve_i + Ki*deltat*Xerr;
Ve = AdT*Vd + Kp*Xerr + Ve_i;
end

