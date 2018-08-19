config_initial    = [0,0,0,-0.1,-0.1,-0.1,-0.1,-0.1,0,0,0,0];
% thetalist_initial = config_initial(4:8)';
% 
% w1 = [0, 0,1];
% w2 = [0,-1,0];
% w3 = [0,-1,0];
% w4 = [0,-1,0];
% w5 = [0, 0,1];
% 
% v1 = [0      ,0.033,0];
% v2 = [-0.5076,0    ,0];
% v3 = [-0.3526,0    ,0];
% v4 = [-0.2176,0    ,0];
% v5 = [0      ,0    ,0];
% 
% B1 =[w1,v1]';
% B2 =[w2,v2]';
% B3 =[w3,v3]';
% B4 =[w4,v4]';
% B5 =[w5,v5]';
% 
% Blist = [B1,B2,B3,B4,B5];
% Tsb = [cos(config_initial(1)),-sin(config_initial(1)),0,config_initial(2);sin(config_initial(1)),cos(config_initial(1)),0,config_initial(3);0,0,1,0.0963;0,0,0,1];
% Tb0 = [1,0,0,0.1662;0,1,0,0;0,0,1,0.0026;0,0,0,1];
% M0e = [1,0,0,0.033;0,1,0,0;0,0,1,0.6546;0,0,0,1];
% T0e = FKinBody(M0e,Blist,thetalist_initial);
% Tse_initial = Tsb*Tb0*T0e;
Tse_initial = [0 0 1 0;0 1 0 0;-1 0 0 0.5;0 0 0 1];
Tsc_initial = [1,0,0,1;0,1,0,0;0,0,1,0.025;0,0,0,1];
Tsc_final    = [0,1,0,0;-1,0,0,-1;0,0,1,0.025;0,0,0,1];

Kp = 1*eye(6);
Ki = 0*eye(6);
[configs,Xerrs] = MobileManipulation(Tse_initial,Tsc_initial,Tsc_final,config_initial,Kp,Ki);
csvwrite('C:\Program Files\V-REP3\V-REP_PRO_EDU\scenes\customscenes\yb1.csv',configs);