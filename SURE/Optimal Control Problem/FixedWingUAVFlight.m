function [problem,guess,phaseoptions] = FixedWingUAVFlight
%KineticBatchReactor - KineticBatchReactor Problem with a multi-phase formulation
%
% The problem was adapted from Example 4.11 from
% J. Betts, "Practical Methods for Optimal Control and Estimation Using Nonlinear Programming: Second Edition," Advances in Design and Control, Society for Industrial and Applied Mathematics, 2010.
%
% Outputs:
%    problem - Structure with information on the optimal control problem
%    guess   - Guess for state, control and multipliers.
%
% Other m-files required: none
% MAT-files required: none
%
% Copyright (C) 2019 Yuanbo Nie, Omar Faqir, and Eric Kerrigan. All Rights Reserved.
% The contribution of Paola Falugi, Eric Kerrigan and Eugene van Wyk for the work on ICLOCS Version 1 (2010) is kindly acknowledged.
% This code is published under the MIT License.
% Department of Aeronautics and Department of Electrical and Electronic Engineering,
% Imperial College London London  England, UK 
% ICLOCS (Imperial College London Optimal Control) Version 2.5 
% 1 Aug 2019
% iclocs@imperial.ac.uk


% Initial and final time for different phases. Let t_min(end)=t_max(end) if
% tf is fixed.
missionData = load("missionData.mat");

%might want to consider reworking these time guesses as they won't
%generalise well to large missions
problem.mp.time.t_min=0.01*ones(1,missionData.nPhases+1);
problem.mp.time.t_min(1,1)=0; 
problem.mp.time.t_max=linspace(0,50*missionData.nPhases,missionData.nPhases+1); 


if exist("preSolve.mat","file")
    preSolve=load("preSolve.mat");
    for i=1:missionData.nPhases
        guess.mp.time(i+1)=preSolve.solution.phaseSol{i}.tf;
    end
    %linkage constraints with chi and waypoint evasion
    %problem.mp.constraints.bll.linear=zeros(1,12*(missionData.nPhases-1)+10);
    %problem.mp.constraints.blu.linear=zeros(1,12.*(missionData.nPhases-1)+10);
    problem.mp.constraints.bll.linear=zeros(1,11*(missionData.nPhases-1)+9);
    problem.mp.constraints.blu.linear=zeros(1,11.*(missionData.nPhases-1)+9);
else
    %this doesn't generalise well either 
    guess.mp.time=linspace(0,15*missionData.nPhases,missionData.nPhases+1);
    
    %linkage constraints without chi and waypoint evasion
    problem.mp.constraints.bll.linear=zeros(1,10*(missionData.nPhases-1)+8);
    problem.mp.constraints.blu.linear=zeros(1,10.*(missionData.nPhases-1)+8);
end

% Parameters bounds. pl=< p <=pu
problem.mp.parameters.pl=[];
problem.mp.parameters.pu=[];
guess.mp.parameters=[];

% Bounds for linkage boundary constraints bll =< bclink(x0,xf,u0,uf,p,t0,tf,vdat) =< blu



problem.mp.constraints.bll.nonlinear=[];
problem.mp.constraints.blu.nonlinear=[];
problem.mp.constraints.blTol.nonlinear=[];

% Get function handles
problem.mp.linkfunctions=@bclink;

% Store the necessary problem parameters used in the functions
auxdata.g=9.81;             %gravitational accelaration
auxdata.ktomps=0.514444;    %conversion factor
auxdata.ftom=0.3048;        %conversion factor
auxdata.R=287.15;           %gas constant
auxdata.ps=101325;          %pressure at sea level
auxdata.rhos=1.225;         %density at sea level
auxdata.Ts=288.15;          %temperature at sea level
auxdata.dTdH=-0.0065;       %temperature gradient
%auxdata.nprop=5827;         %proppelor rotational speed [rpm] (MAX)#
auxdata.NumberProps=2;       %number of propellers
auxdata.Dprop=0.5588;         %propellor diameter [m]
auxdata.kappa=1.4;          %heat capacity ratio
%auxdata.alpha0=-11.16*pi/180;%zero lift angle of attack [rad]
auxdata.alpha0=-3.488*pi/180;%zero lift angle of attack [rad]
auxdata.clalpha=0.086;    %lift gradietn [deg^-1]
%auxdata.k_cd=0.05;      %k value for drag
auxdata.k_cd=0.05;
%auxdata.cd0=0.8;        %zerolift drag coefficient
auxdata.cd0=0.8;
auxdata.S=0.752;              %wing surface area [m^2]
auxdata.alphat=0*pi/180;    %thrust angle [rad] 
%battery circuit parameters
auxdata.Q=16*3600; %Pack capacity in coulombs (converted from Ah)
auxdata.R0=0.003; % Thevenin internal resistance
auxdata.R1=0.002; % Thevenin RC branch
auxdata.C1=8000;  % %Thevenin RC branch
auxdata.Series = 6; % Number of cells in series
auxdata.CRating = 30; %Battery discharge rating
auxdata.OCV_Curve = @(SOC) (3.64 +0.55.*SOC - 0.72.*SOC.^2 + 0.75.*SOC.^3);
auxdata.MaxCurrentPerMotor = 65;
%wind parameters
auxdata.WindSpeed = 6.43; % 
auxdata.WindHeading=-45*pi/180; % Direction wind is coming *from*, e.g. a westerly would be -pi/2
auxdata.OEW = 9.2*auxdata.g; %Operating empty weight (N)
auxdata.WPayload = 0*auxdata.g; %Payload weight (N)


% auxdata.FFModel=FFModel;


%Lookup tables

%Cl-alpha only linear region
auxdata.CL_Data_Fit=griddedInterpolant([auxdata.alpha0 12*pi/180],[0 auxdata.clalpha*(12-auxdata.alpha0*180/pi)],'spline');

nProp_Data = readtable("PowerRPMLookup.csv");
auxdata.nProp_Data_Fit=griddedInterpolant(nProp_Data.Power,nProp_Data.RPM,'spline');

% advance ratio - efficiency
ita_Data=[0	0.0001
        0.05	0.1447
        0.1	0.2468
        0.15	0.3538
        0.2	0.448
        0.25	0.5281
        0.3	0.5962
        0.35	0.6516
        0.4	0.7008
        0.45	0.7405
        0.5	0.7721
        0.55	0.7943
        0.6	0.8045
        0.65	0.7967
        0.66	0.7918
        0.67	0.7852
        0.68	0.7767
        0.69	0.7657
        0.7	0.7516
        0.71	0.7336
        0.72	0.7101
        0.73	0.679
        0.74	0.6372
        0.75	0.5784
        0.76	0.4921
        0.77	0.3539
        0.78	0.1023
        0.79	0
        0.8	0
        1	0];
auxdata.ita_Data_Fit=griddedInterpolant(ita_Data(:,1),ita_Data(:,2),'spline');

problem.mp.data.auxdata=auxdata;

% Define different phases of OCP
phaseoptions=cell(1,missionData.nPhases);
for i=1:missionData.nPhases
    [problem.phases{i},guess.phases{i}] = FixedWingUAVFlight_PhaseN(problem.mp, guess.mp, i);
    phaseoptions{i}=problem.phases{i}.settings(missionData.phaseNodes(i));
end

%------------- END OF CODE --------------


function [blc_linear, blc_nonlinear]=bclink(x0,xf,u0,uf,p,t0,tf,vdat)

% bclink - Returns the evaluation of the linkage boundary constraints: bll =< bclink(x0,xf,u0,uf,p,t0,tf,vdat) =< blu
%
% Syntax:  [blc_linear, blc_nonlinear]=bclink(x0,xf,u0,uf,p,t0,tf,vdat)
%
% Inputs:
%    x0  - state at t=0
%    xf  - state at t=tf
%    u0  - input at t=0
%    uf  - input at t=tf
%    p   - parameter
%    tf  - final time
%    vdat- structured variable containing the values of additional data used inside
%          the function
%
%          
% Output:
%    blc_linear - column vector containing the evaluation of the linear linkage boundary constraint functions
%    blc_nonlinear - column vector containing the evaluation of the nonlinear linkage boundary constraint functions
%
%------------- BEGIN CODE --------------

missionData=load("missionData.mat");
%ensure dimensions of blc_linear and bll/blu line up (lines 37-43)
%ensure dimensions of indexing below line up as well
if exist("preSolve.mat","file")
    %blc_linear=zeros(12.*(missionData.nPhases-1)+10,1);
    blc_linear=zeros(11.*(missionData.nPhases-1)+9,1);
    for i=1:(missionData.nPhases)
        %[a,b,c] = computeWaypointEvasion (i,missionData.navigation.waypoints.cartesian,[5 20]);
        if i == missionData.nPhases
            blc_linear(1+11*(i-1):9+11*(i-1)) = [xf{i}(1) - x0{1}(1); xf{i}(2) - x0{1}(2); xf{i}(3) - x0{1}(3); xf{i}(4) - x0{1}(4); xf{i}(5) - x0{1}(5); mod(xf{i}(6),2*pi) - mod(x0{1}(6), 2*pi); uf{i}(1) - u0{1}(1); uf{i}(2) - u0{1}(2); uf{i}(3) - u0{1}(3);];
            %blc_linear(1+12*(i-1):10+12*(i-1)) = [xf{i}(1) - x0{1}(1); xf{i}(2) - x0{1}(2); xf{i}(3) - x0{1}(3); xf{i}(4) - x0{1}(4); xf{i}(5) - x0{1}(5); mod(xf{i}(6),2*pi) - mod(x0{1}(6), 2*pi); uf{i}(1) - u0{1}(1); uf{i}(2) - u0{1}(2); uf{i}(3) - u0{1}(3);a.*xf{i}(2) + b.*xf{i}(3)-c;];
        else
            blc_linear(1+11*(i-1):11*i) = [xf{i}(1) - x0{i+1}(1); xf{i}(2) - x0{i+1}(2); xf{i}(3) - x0{i+1}(3); xf{i}(4) - x0{i+1}(4); xf{i}(5) - x0{i+1}(5); xf{i}(6) - x0{i+1}(6); xf{i}(7) - x0{i+1}(7);  xf{i}(8) - x0{i+1}(8); uf{i}(1) - u0{i+1}(1); uf{i}(2) - u0{i+1}(2); uf{i}(3) - u0{i+1}(3);];
            %blc_linear(1+12*(i-1):12*i) = [xf{i}(1) - x0{i+1}(1); xf{i}(2) - x0{i+1}(2); xf{i}(3) - x0{i+1}(3); xf{i}(4) - x0{i+1}(4); xf{i}(5) - x0{i+1}(5); xf{i}(6) - x0{i+1}(6); xf{i}(7) - x0{i+1}(7);  xf{i}(8) - x0{i+1}(8); uf{i}(1) - u0{i+1}(1); uf{i}(2) - u0{i+1}(2); uf{i}(3) - u0{i+1}(3);a.*xf{i}(2) + b.*xf{i}(3)-c;];
        end
    end
else
    blc_linear=zeros(10.*(missionData.nPhases-1)+8,1);
    for i=1:(missionData.nPhases)
        if i == missionData.nPhases
            blc_linear(1+10*(i-1):8+10*(i-1)) = [xf{i}(1) - x0{1}(1); xf{i}(2) - x0{1}(2); xf{i}(3) - x0{1}(3); xf{i}(4) - x0{1}(4); xf{i}(5) - x0{1}(5); uf{i}(1) - u0{1}(1); uf{i}(2) - u0{1}(2); uf{i}(3) - u0{1}(3);];
        else
            blc_linear(1+10*(i-1):10*i) = [xf{i}(1) - x0{i+1}(1); xf{i}(2) - x0{i+1}(2); xf{i}(3) - x0{i+1}(3); xf{i}(4) - x0{i+1}(4); xf{i}(5) - x0{i+1}(5); xf{i}(7) - x0{i+1}(7);  xf{i}(8) - x0{i+1}(8); uf{i}(1) - u0{i+1}(1); uf{i}(2) - u0{i+1}(2); uf{i}(3) - u0{i+1}(3);];
        end
    end
end

blc_nonlinear=[];
%------------- END OF CODE --------------
