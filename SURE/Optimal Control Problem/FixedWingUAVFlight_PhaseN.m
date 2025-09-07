function [problem,guess] = FixedWingUAVFlight_PhaseN(problem_mp, guess_mp, phaseNumber)
%KineticBatchReactor - KineticBatchReactor Problem (Phase 1) 
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

% variable bounds
missionData=load("missionData.mat");
if exist("preSolve.mat","file")
    chi_max=missionData.navigation.chiGuess(phaseNumber)+pi;
    chi_min=missionData.navigation.chiGuess(phaseNumber)-pi;
    if phaseNumber ==1
        [~,~,~,xf_min,xf_max,yf_min,yf_max]=computeWaypointEvasion(1,missionData.navigation.waypoints.cartesian,missionData.bounds.mu);
        [~,~,~,x0_min,x0_max,y0_min,y0_max]=computeWaypointEvasion(missionData.nPhases,missionData.navigation.waypoints.cartesian,[5 20]);
    else
        [~,~,~,xf_min,xf_max,yf_min,yf_max]=computeWaypointEvasion(phaseNumber,missionData.navigation.waypoints.cartesian,[5 20]);
        [~,~,~,x0_min,x0_max,y0_min,y0_max]=computeWaypointEvasion(phaseNumber-1,missionData.navigation.waypoints.cartesian,[5 20]);
    end
else
    chi_max=pi;
    chi_min=-pi;
    if phaseNumber ==1
        [~,~,~,xf_min,xf_max,yf_min,yf_max]=computeWaypointEvasion(1,missionData.navigation.waypoints.cartesian,[0 0]);
        [~,~,~,x0_min,x0_max,y0_min,y0_max]=computeWaypointEvasion(missionData.nPhases,missionData.navigation.waypoints.cartesian,[0 0]);
    else
        [~,~,~,xf_min,xf_max,yf_min,yf_max]=computeWaypointEvasion(phaseNumber,missionData.navigation.waypoints.cartesian,[0 0]);
        [~,~,~,x0_min,x0_max,y0_min,y0_max]=computeWaypointEvasion(phaseNumber-1,missionData.navigation.waypoints.cartesian,[0 0]);
    end
end

% xf=missionData.navigation.waypoints.cartesian.DistNorth(phaseNumber+1); %Initial position north
% yf=missionData.navigation.waypoints.cartesian.DistEast(phaseNumber+1);  %Initial position east

%------------- BEGIN CODE --------------
% Plant model name, used for Adigator and simulation
InternalDynamics=@FixedWingUAVFlight_Dynamics_Internal;
SimDynamics=[];

% Analytic derivative files (optional)
problem.analyticDeriv.gradCost=[];
problem.analyticDeriv.hessianLagrangian=[];
problem.analyticDeriv.jacConst=[];

% Settings file
problem.settings=@settings_FixedWingUAVFlight_PhaseN;

% Initial time. t0<tf
problem.time.t0_idx=phaseNumber;
problem.time.t0_min=problem_mp.time.t_min(problem.time.t0_idx);
problem.time.t0_max=problem_mp.time.t_max(problem.time.t0_idx);
guess.t0=guess_mp.time(problem.time.t0_idx);

% Final time. Let tf_min=tf_max if tf is fixed.
problem.time.tf_idx=phaseNumber+1;
problem.time.tf_min=problem_mp.time.t_min(problem.time.tf_idx);     
problem.time.tf_max=problem_mp.time.t_max(problem.time.tf_idx); 
guess.tf=guess_mp.time(problem.time.tf_idx);

% Parameters bounds. pl=< p <=pu
problem.parameters.pl=problem_mp.parameters.pl;
problem.parameters.pu=problem_mp.parameters.pu;
guess.parameters=guess_mp.parameters;


% State bounds. xl=< x <=xu
problem.states.xl=missionData.bounds.xl;
problem.states.xl(6)=chi_min;

problem.states.xu=missionData.bounds.xu;
problem.states.xu(6)=chi_max;

% Initial conditions for system.
problem.states.x0=[];
% Initial conditions for system. Bounds if x0 is free s.t. x0l=< x0 <=x0u

%could speed this up by computing bounds on xf from previous phase
%(computeWaypointEvasion)
problem.states.x0l=problem.states.xl;
problem.states.x0l(2) = x0_min;
problem.states.x0l(3) = y0_min;

problem.states.x0u=problem.states.xu;
problem.states.x0u(2) = x0_max;
problem.states.x0u(3) = y0_max;

%battery charge must be at a maximum at the start, rc
if phaseNumber==1
    problem.states.x0l(7)=problem.states.x0u(7);
end

% State rate bounds. xrl=< x <=xru
% problem.states.xrl=[-inf -inf -inf -inf -inf -inf -inf]; 
% problem.states.xru=[inf inf inf inf inf inf inf]; 

% State error bounds
%Not sure about the values I've put for SOC and Volt_RC
problem.states.xErrorTol_local=[5 5 5 1 deg2rad(5) deg2rad(5) 0.01 0.01];
problem.states.xErrorTol_integral=[5 5 5 1 deg2rad(5) deg2rad(5) 0.01 0.01];

% State constraint error bounds
problem.states.xConstraintTol=[1 1 1 0.5 deg2rad(5) deg2rad(5) 0.01 0.01];
problem.states.xrConstraintTol=[1 1 1 0.5 deg2rad(5) deg2rad(5) 0.01 0.01];

% Terminal state bounds. xfl=< xf <=xfu

problem.states.xfl=missionData.bounds.xl;
problem.states.xfl(2)=xf_min;
problem.states.xfl(3)=yf_min;

problem.states.xfu=missionData.bounds.xu;
problem.states.xfu(2)=xf_max;
problem.states.xfu(3)=yf_max;

%edit heading bounds as they are specific for each phase
if exist("preSolve.mat","file") && phaseNumber==missionData.nPhases
    problem.states.xfl(6)=missionData.navigation.chi_f-pi;
    problem.states.xfu(6)=missionData.navigation.chi_f+pi;
else
    problem.states.xfu(6)=chi_max;
    problem.states.xfl(6)=chi_min;
end


% Number of control actions N 
% Set problem.inputs.N=0 if N is equal to the number of integration steps.  
% Note that the number of integration steps defined in settings.m has to be divisible 
% by the  number of control actions N whenever it is not zero.
problem.inputs.N=0;       
      
% Input bounds
problem.inputs.ul=missionData.bounds.ul;
problem.inputs.uu=missionData.bounds.uu;

problem.inputs.u0l=missionData.bounds.ul;
problem.inputs.u0u=missionData.bounds.uu;

% Input rate bounds
% problem.inputs.url=[-deg2rad(2) -deg2rad(10) -Pmax/5];
% problem.inputs.uru=[deg2rad(2) deg2rad(10) Pmax/5];

% Input constraint error bounds
problem.inputs.uConstraintTol=[deg2rad(0.5) deg2rad(0.5) 0.1];
problem.inputs.urConstraintTol=[deg2rad(0.5) deg2rad(0.5) 0.1];

%Guess states and inputs

if exist("preSolve.mat","file")
    preSolve=load("preSolve.mat");
    if phaseNumber<=length(preSolve.solution.phaseSol)
        guess.time=preSolve.solution.phaseSol{phaseNumber}.T;
        guess.states=preSolve.solution.phaseSol{phaseNumber}.X;
        guess.inputs=preSolve.solution.phaseSol{phaseNumber}.U;
        guess.states(:,6) = missionData.navigation.chiGuess(phaseNumber).*ones(size(guess.states(:,6))); % Set track angle guess to prevent inefficient turns
    end
else
    %ramp based on euclidean distance and mean velocity
    V_tas_mean = (missionData.bounds.xl(4)+missionData.bounds.xu(4))/2;
    if phaseNumber == 1
        t0_estimate = 0;
    else
        t0_estimate = sum(missionData.navigation.distances(1:missionData.nPhases-1));
    end
    tf_estimate = t0_estimate+missionData.navigation.distances(phaseNumber)/V_tas_mean;
    guess.time = linspace(t0_estimate,tf_estimate,missionData.phaseNodes(phaseNumber));
    meanState = (problem.states.xu+problem.states.xl)./2;
    meanInput = (problem.inputs.uu+problem.inputs.ul)./2;
    guess.states = ones(missionData.phaseNodes(phaseNumber),1)*meanState;
    guess.inputs = ones(missionData.phaseNodes(phaseNumber),1)*meanInput;
end
% Choose the set-points if required
problem.setpoints.states=[];
problem.setpoints.inputs=[];

% Bounds for path constraint function gl =< g(x,u,p,t) =< gu

problem.constraints.ng_eq=0;
problem.constraints.gTol_eq=[];

problem.constraints.gl=[];
problem.constraints.gu=[];
problem.constraints.gTol_neq=[];



% Bounds for boundary constraints bl =< b(x0,xf,u0,uf,p,t0,tf) =< bu
problem.constraints.bl=0.01;
problem.constraints.bu=inf;
problem.constraints.bTol=0.01;



% store the necessary problem parameters used in the functions
% problem.data = []; % [lb]
problem.data=problem_mp.data;

% problem.data.tau = tau;
% Get function handles and return to Main.m
problem.data.InternalDynamics=InternalDynamics;
problem.data.functionfg=@fg;
problem.data.plantmodel = func2str(InternalDynamics);
problem.functions={@L,@E,@f,@g,@avrc,@b};
problem.sim.functions=SimDynamics;
problem.sim.inputX=[];
problem.sim.inputU=1:length(problem.inputs.ul);

if (phaseNumber == missionData.nPhases) && (missionData.objective=="Time")
    problem.functions_unscaled={@L_unscaled,@E_unscaled_final_time,@f_unscaled,@g_unscaled,@avrc,@b_unscaled};
elseif (phaseNumber == missionData.nPhases) && (missionData.objective=="Energy")
    problem.functions_unscaled={@L_unscaled,@E_unscaled_final_energy,@f_unscaled,@g_unscaled,@avrc,@b_unscaled};
elseif (phaseNumber == missionData.nPhases) && (missionData.objective=="Charge")
    problem.functions_unscaled={@L_unscaled,@E_unscaled_final_charge,@f_unscaled,@g_unscaled,@avrc,@b_unscaled};
elseif (phaseNumber == missionData.nPhases) && (missionData.objective=="Combined")
    problem.functions_unscaled={@L_unscaled,@E_unscaled_combined,@f_unscaled,@g_unscaled,@avrc,@b_unscaled};
elseif (phaseNumber ~= missionData.nPhases)
    problem.functions_unscaled={@L_unscaled,@E_unscaled_intermediate,@f_unscaled,@g_unscaled,@avrc,@b_unscaled};
end
problem.data.functions_unscaled=problem.functions_unscaled;
problem.data.ng_eq=problem.constraints.ng_eq;
problem.constraintErrorTol=[problem.constraints.gTol_eq,problem.constraints.gTol_neq,problem.constraints.gTol_eq,problem.constraints.gTol_neq,problem.states.xConstraintTol,problem.states.xConstraintTol,problem.inputs.uConstraintTol,problem.inputs.uConstraintTol];

%------------- END OF CODE --------------

function stageCost=L_unscaled(x,xr,u,ur,p,t,vdat)

% L_unscaled - Returns the stage cost.
% The function must be vectorized and
% xi, ui are column vectors taken as x(:,i) and u(:,i) (i denotes the i-th
% variable)
% 
% Syntax:  stageCost = L(x,xr,u,ur,p,t,data)
%
% Inputs:
%    x  - state vector
%    xr - state reference
%    u  - input
%    ur - input reference
%    p  - parameter
%    t  - time
%    data- structured variable containing the values of additional data used inside
%          the function
%
% Output:
%    stageCost - Scalar or vectorized stage cost
%
%  Remark: If the stagecost does not depend on variables it is necessary to multiply
%          the assigned value by t in order to have right vector dimesion when called for the optimization. 
%          Example: stageCost = 0*t;

%------------- BEGIN CODE --------------

computeStageCost;

%------------- END OF CODE --------------


% E_unscaled - Returns the boundary value cost
%
% Syntax:  boundaryCost=E(x0,xf,u0,uf,p,tf,data)
%
% Inputs:
%    x0  - state at t=0
%    xf  - state at t=tf
%    u0  - input at t=0
%    uf  - input at t=tf
%    p   - parameter
%    tf  - final time
%    data- structured variable containing the values of additional data used inside
%          the function
%
% Output:
%    boundaryCost - Scalar boundary cost
%

function boundaryCost=E_unscaled_intermediate(x0,xf,u0,uf,p,t0,tf,vdat) 
%------------- BEGIN CODE --------------

boundaryCost=0;

%------------- END OF CODE --------------

function boundaryCost=E_unscaled_final_time(x0,xf,u0,uf,p,t0,tf,vdat) 

%------------- BEGIN CODE --------------

boundaryCost=tf;

%------------- END OF CODE --------------

function boundaryCost=E_unscaled_final_energy(x0,xf,u0,uf,p,t0,tf,vdat)

%------------- BEGIN CODE --------------
auxdata = vdat.auxdata;
missionData=load("missionData.mat");
Volt0_0 = feval(auxdata.OCV_Curve,missionData.bounds.xu(7).*ones(size(xf(7))));
Volt0_f = feval(auxdata.OCV_Curve,xf(7));
%Volt_Out0 = auxdata.Series.*(Volt0_0 + x0(8) + auxdata.R0.*u0(3));
%Volt_Outf = auxdata.Series.*(Volt0 + xf(8) + auxdata.R0.*uf(3));
% penalise difference between starting energy and final energy
% energy in Wh
boundaryCost= auxdata.Series.*auxdata.Q.*(Volt0_0.*missionData.bounds.xu(7).*ones(size(xf(7)))-Volt0_f.*xf(7))./3600;
%------------- END OF CODE --------------

function boundaryCost=E_unscaled_combined(x0,xf,u0,uf,p,t0,tf,vdat)

%------------- BEGIN CODE --------------
auxdata = vdat.auxdata;
missionData=load("missionData.mat");
Volt0_0 = feval(auxdata.OCV_Curve,missionData.bounds.xu(7).*ones(size(xf(7))));
Volt0_f = feval(auxdata.OCV_Curve,xf(7));
%Volt_Out0 = auxdata.Series.*(Volt0_0 + x0(8) + auxdata.R0.*u0(3));
%Volt_Outf = auxdata.Series.*(Volt0 + xf(8) + auxdata.R0.*uf(3));
% penalise difference between starting energy and final energy
% energy in Wh
finalEnergy= auxdata.Series.*auxdata.Q.*(Volt0_0.*missionData.bounds.xu(7).*ones(size(xf(7)))-Volt0_f.*xf(7))./3600;
boundaryCost = 2*finalEnergy + finalEnergy;
%------------- END OF CODE --------------

function boundaryCost=E_unscaled_final_charge(x0,xf,u0,uf,p,t0,tf,vdat) 

%------------- BEGIN CODE --------------

boundaryCost=-xf(7);

%------------- END OF CODE --------------

function bc=b_unscaled(x0,xf,u0,uf,p,t0,tf,vdat,varargin)

% b_unscaled - Returns a column vector containing the evaluation of the boundary constraints: bl =< bf(x0,xf,u0,uf,p,t0,tf) =< bu
%
% Syntax:  bc=b(x0,xf,u0,uf,p,tf,data)
%
% Inputs:
%    x0  - state at t=0
%    xf  - state at t=tf
%    u0  - input at t=0
%    uf  - input at t=tf
%    p   - parameter
%    tf  - final time
%    data- structured variable containing the values of additional data used inside
%          the function
%
%          
% Output:
%    bc - column vector containing the evaluation of the boundary function 
%
%------------- BEGIN CODE --------------
varargin=varargin{1};
bc=tf-t0;
%------------- END OF CODE --------------
% When adpative time interval add constraint on time
%------------- BEGIN CODE --------------
if length(varargin)==2
    options=varargin{1};
    t_segment=varargin{2};
    if ((strcmp(options.discretization,'hpLGR')) || (strcmp(options.discretization,'globalLGR')))  && options.adaptseg==1 
        if size(t_segment,1)>size(t_segment,2)
            bc=[bc;diff(t_segment)];
        else
            bc=[bc;diff(t_segment)'];
        end
    end
end

%------------- END OF CODE --------------

