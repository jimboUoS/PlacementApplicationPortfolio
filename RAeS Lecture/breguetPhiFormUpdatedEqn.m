%physical constants
g=9.81;

nPhiPoints = 1001;
nMassEnergyPoints = 1001;

%aircraft params

%efficiencies
PSFC = 0.0000001113484452;

liftDragRatio=13.5;

etaProp=0.8;
etaTurbine = 0.35;
etaGenerator = 0.98;
etaMotor = 0.9;
etaGearbox = 0.95;

etaFuelChain = etaTurbine*etaGearbox*etaGenerator*etaMotor*etaProp;
etaBatChain = etaGearbox*etaMotor*etaProp;

%mission requirements
massPayload=1200;
rangeLowerBound=500 *10^3;

%masses

massOperatingEmpty=2145;


maxMTOW=1.75*3629;

%specific energies/consumption
eBat=0.8*500*3600;
eFuel = 43.1*10^6;

%bounds on fuel mass
deltaMConventional= 3629*(1-exp(-(rangeLowerBound*g*PSFC)/(etaProp*liftDragRatio)));

%the energy mass must be at least equal to the conventional fuel deltaM
%but must not exceed MTOW-payload-oem

%dependent variables
phi=linspace(0,1,nPhiPoints);
massEnergy = linspace(deltaMConventional,(maxMTOW-massOperatingEmpty-massPayload),nMassEnergyPoints)';


%independent variables
massFuel = (((1-phi).*eBat.*massEnergy)./(phi.*eFuel+(1-phi).*eBat));
normalisedFuelMass = (1./deltaMConventional).*massFuel;


MTOW=massPayload+massOperatingEmpty+massEnergy;

normalisedMTOW = MTOW./3629;

range = (1/g).*liftDragRatio.*( etaFuelChain.*eFuel.*log((MTOW)./(MTOW-massFuel)) + etaBatChain.*(eBat.*(massEnergy-massFuel))./(MTOW-massFuel));

%normalisedFuelMass = phi.*ones([100 1]);

%two criteria for feasibility
%1. consumed fuel mass is lower than a conventional equivalent
%2. range can be achieved with given energy mix

%remove all points that have a percent fuel saving less than 0 or that
%don't meet the range requirement
for i = 1:nPhiPoints
    for  j = 1:nMassEnergyPoints
        if (normalisedFuelMass(j,i) > 1)
           normalisedFuelMass(j,i) = NaN;
           %fprintf("Target: %f\tActual: %f\n\n",range,modifiedBreguetRange(phi(i), massEnergy(j), etaFuelChain, etaBatChain, liftDragRatio, eBat, eFuel, g, (massEnergy(j)+massPayload+massOperatingEmpty)));
        elseif (range(j,i) < rangeLowerBound)
        %if (range(j,i) < rangeLowerBound)
            normalisedFuelMass(j,i) = NaN;
            %fprintf("Target: %f\tActual: %f\n\n",range,modifiedBreguetRange(phi(i), massEnergy(j), etaFuelChain, etaBatChain, liftDragRatio, eBat, eFuel, g, (massEnergy(j)+massPayload+massOperatingEmpty)));
        end
    end
end

percentFuelSaving = (1-normalisedFuelMass)*100;

figure(1);
surf(phi,normalisedMTOW,percentFuelSaving,"EdgeColor","none");
xlabel("Degree of Hybridisation");
ylabel("Normalised MTOW");
zlabel("Percent Fuel Saving (%)");
view(3);
saveas(figure(1),"Figures\Surface\PercentFuelSaving","jpg")

figure(2);
surf(phi,normalisedMTOW,range*10^-3,"EdgeColor","none");
xlabel("Degree of Hybridisation");
ylabel("Normalised MTOW");
zlabel("Range (km)");
saveas(figure(2),"Figures\Surface\Range","jpg")

% figure(3);
% contour(phi,massEnergy,range*10^-3);
% xlabel("Degree of Hybridisation");
% ylabel("Mass of Energy (kg)");
% zlabel("Range (km)");

minFC = min(min(normalisedFuelMass));
indexOfMinFC = find(normalisedFuelMass==minFC);

fprintf("Minimum normalised fuel consumption: %f\nAchieved at phi=%f Energy Storage Mass = %f kg\n",minFC, phi(fix(indexOfMinFC/nPhiPoints)), massEnergy(mod(indexOfMinFC,nMassEnergyPoints)));

% function R = modifiedBreguetRange(phi, massEnergy, etaFuelChain, etaBatChain, liftDragRatio, eBat, eFuel, g, MTOW)
%     massFuel = ((1-phi)*eBat*massEnergy)/(phi*eFuel+(1-phi)*eBat);
%     %massBat = massEnergy-massFuel;
% 
%     R=(1/g)*liftDragRatio*(etaFuelChain*eFuel*log(MTOW/(MTOW-massFuel) + etaBatChain*(eBat*(massEnergy-massFuel))/(MTOW-massFuel)) );
% end


