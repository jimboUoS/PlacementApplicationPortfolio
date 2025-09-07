function bounds =  defineBounds
    load("auxdata.mat","auxdata");
    
    H_min=137+30; %min altitude
    H_max=137+50; %max altitude
    W=auxdata.OEW+auxdata.WPayload;
    
    alpha_max=10*pi/180; %max angle of attack
    alpha_min=-10*pi/180; %min angle of attack
    gamma_max=45*pi/180; %max flight path angle
    gamma_min=-45*pi/180; %min flight path angle 
    CL_max=auxdata.CL_Data_Fit(alpha_max); %max lift coefficient
    Vtas_min=sqrt(2*W/auxdata.rhos/auxdata.S/CL_max); %min TAS (occur at min weight, max density and max CL)
    Vtas_max=20; %max TAS
    % Vtas_min=16;
    % Vtas_max=16.5;
    
    SOC_max = 1;
    SOC_min = 0;
    
    Volt_RC_max=0;
    Volt_RC_min=-0.25;
    
    curr_min = max(-auxdata.CRating*auxdata.Q/3600, -auxdata.MaxCurrentPerMotor.*auxdata.NumberProps);
    curr_max = -readtable("PowerRPMLookup.csv").Power(1).*auxdata.NumberProps./(auxdata.Series.*feval(auxdata.OCV_Curve,SOC_min));
    x_min = -1e4;
    y_min = -1e4;
    x_max=1e4;
    y_max=1e4;

    %bounds on chi must still be calculated in PhaseN file
    chi_max=pi;
    chi_min=-pi;
    
    phi_max=45*pi/180;
    
    load("missionData.mat","bounds")
    bounds.xu = [H_max x_max y_max Vtas_max gamma_max chi_max SOC_max Volt_RC_max];
    bounds.xl = [H_min x_min y_min Vtas_min gamma_min chi_min SOC_min Volt_RC_min];
    bounds.uu = [alpha_max phi_max curr_max];
    bounds.ul = [alpha_min -phi_max curr_min];
    bounds.mu = [5 20];
end