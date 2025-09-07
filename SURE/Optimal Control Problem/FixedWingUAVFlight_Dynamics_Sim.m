function dx = FixedWingUAVFlight_Dynamics_Sim(x,u,p,t,data)
% Aircraft Dynamics - Simulation
%
% Syntax:  
%          [dx] = Dynamics(x,u,p,t,vdat)
% 
% Inputs:
%    x  - state vector
%    u  - input
%    p  - parameter
%    t  - time
%    vdat - structured variable containing the values of additional data used inside
%          the function%      
% Output:
%    dx - time derivative of x
%
% Copyright (C) 2019 Yuanbo Nie, Omar Faqir, and Eric Kerrigan. All Rights Reserved.
% The contribution of Paola Falugi, Eric Kerrigan and Eugene van Wyk for the work on ICLOCS Version 1 (2010) is kindly acknowledged.
% This code is published under the MIT License.
% Department of Aeronautics and Department of Electrical and Electronic Engineering,
% Imperial College London London  England, UK 
% ICLOCS (Imperial College London Optimal Control) Version 2.5 
% 1 Aug 2019
% iclocs@imperial.ac.uk
auxdata = data.auxdata;

H = x(:,1);
npos = x(:,2);
epos = x(:,3);
V_tas = x(:,4);
gamma = x(:,5);
chi = x(:,6);
SOC = x(:,8);
Volt_RC = x(:,9);

alpha = u(:,1);
phi = u(:,2);
current = u(:,3);

%Battery dynamics
SOC_dot=current./(auxdata.Q);
Volt_RC_dot = -Volt_RC/(auxdata.R1*auxdata.C1) + current/(auxdata.C1);
Volt0 = feval(auxdata.OCV_Curve,SOC);
Volt_Out = auxdata.Series.*(Volt0 + Volt_RC + auxdata.R0.*current);

Temp=auxdata.Ts+H.*auxdata.dTdH;
% pressure=auxdata.ps*(Temp./auxdata.Ts).^(-auxdata.g/auxdata.dTdH/auxdata.R);
rho=auxdata.rhos.*(Temp./auxdata.Ts).^(-(auxdata.g./auxdata.dTdH./auxdata.R+1));
rho_ratio=rho./auxdata.rhos;
% a_sos=sqrt(auxdata.kappa*auxdata.R.*Temp);

%Electrical power input defined with negative current as sign convention
%Refer to Courtier et al 2022 "Discretisation-free battery fast-charging optimisation using the 
% measure-moment approach"

P=Volt_Out.*(-current);
Ph=(1.132.*rho_ratio-0.132).*P; %power at altitude
J=60*V_tas./auxdata.nProp_Data_Fit(P./auxdata.NumberProps)./auxdata.Dprop;
ita=auxdata.ita_Data_Fit(J);
Thrust=ita.*Ph./V_tas;

% Calculate aerodynamic forces
cl=auxdata.CL_Data_Fit(alpha);
L=0.5.*cl.*rho.*V_tas.^2.*auxdata.S;
cd=auxdata.cd0+auxdata.k_cd*cl.^2;
Drag=0.5.*cd.*rho.*V_tas.^2.*auxdata.S;

%% equations of motions
W=auxdata.OEW+auxdata.WPayload;

TAS_dot=(Thrust-Drag-W.*sin(gamma)).*auxdata.g./W;
gamma_dot=(L.*cos(phi)+Thrust*sin(auxdata.alphat).*cos(phi)-W.*cos(gamma))*auxdata.g./W./V_tas;
H_dot=V_tas.*sin(gamma);
WindNorth = -auxdata.WindSpeed.*cos(auxdata.WindHeading).*ones(size(V_tas));
WindEast = -auxdata.WindSpeed.*sin(auxdata.WindHeading).*ones(size(V_tas));
%ground speed
x_dot=V_tas.*cos(gamma).*cos(chi) + WindNorth;
y_dot=V_tas.*cos(gamma).*sin(chi) + WindEast;
chi_dot=L.*sin(phi)./cos(gamma)*auxdata.g./W./V_tas;
% Mach=V_tas./a_sos;

%% 
dx = [H_dot, x_dot, y_dot, TAS_dot, gamma_dot, chi_dot, SOC_dot,Volt_RC_dot];

% load("missionData.mat","navigation");
% 
% [aGeofence, bGeofence, cGeofence] = geofenceToIneqConstraints(navigation.geofence.cartesian);
% 
% g_neq = aGeofence.*npos + bGeofence.*epos + cGeofence;

