displayOutput = false;

%physical constants
g=9.81;
%aircraft params

%efficiencies
PSFC = [0.0000001113484452 0.00000007789322188 0.0000000790759823];

liftDragRatio=[13.5 14 16.8];

etaProp=0.8;
etaTurbine = 0.35;
etaGenerator = 0.98;
etaMotor = 0.9;
etaGearbox = 0.95;

etaFuelChain = etaTurbine*etaGearbox*etaGenerator*etaMotor*etaProp;
etaBatChain = etaGearbox*etaMotor*etaProp;

%mission requirements
massPayload=[1200, 3298, 7400];

%masses

massOperatingEmpty=[2145 8620 13600];

aircraftMTOW=[3629 13155 23000];

aircraftNames = ["Cessna 208 Caravan" "SAAB 340B" "ATR72-600"];

%specific energies/consumption
eBat = 0.8*3600*linspace(300,900,4);
eFuel = 43.1*10^6;



%the energy mass must be at least equal to the conventional fuel deltaM
%but must not exceed MTOW-payload-oem

%dependent variables
nRangePoints = 701;


for i =1:3
    mkdir(strcat("Figures\",aircraftNames(i)))
end


for q = 1:4
    for i=1:3
        rangeConventional = etaProp*liftDragRatio(i)*log(aircraftMTOW(i)/(massPayload(i)+massOperatingEmpty(i)))/(g*PSFC(i));
        rangeLowerBound=linspace(0,rangeConventional,nRangePoints);
        for MTOWMultiplier = linspace(1,2,5)
            for j = 1:nRangePoints
                %bounds on fuel mass
                deltaMConventional=(aircraftMTOW(i))*(1-exp(-g*PSFC(i)*rangeLowerBound(j)/(liftDragRatio(i)*etaProp)));
                
                maxMTOW=MTOWMultiplier*aircraftMTOW(i);
                massEnergy = maxMTOW - massPayload(i) - massOperatingEmpty(i);
                
                phiUpper = 1;
                phiLower = 0;
                
                for k=1:16
                    phi = (phiUpper+phiLower)/2;
                    massFuel = (((1-phi).*eBat(q).*massEnergy)./(phi.*eFuel+(1-phi).*eBat(q)));
                    range = (1/g)*liftDragRatio(i)*( etaFuelChain*eFuel*log((maxMTOW)/(maxMTOW-massFuel)) + etaBatChain*(eBat(q)*(massEnergy-massFuel))/(maxMTOW-massFuel));
                    if range>=rangeLowerBound(j)
                        phiLower=phi;
                    elseif range<rangeLowerBound(j)
                        phiUpper = phi;
                    end
                    normalisedFuelMass = massFuel/deltaMConventional;
                end
    
                if displayOutput
                    fprintf("Normalised Fuel Consumption Minimised at phi=%f\n",phi);
                    fprintf("\tNFC = %f\n",normalisedFuelMass);
                    fprintf("\tRange = %.0f km\n",range*10^-3);
                end
                
                percentFuelSaving(j)=(1-normalisedFuelMass)*100;
    
            end
            figure(3*(q-1)+i);
            plot(rangeLowerBound*10^-3,percentFuelSaving,"DisplayName",strcat("MTOW = ",string(MTOWMultiplier),"x"),"LineWidth",3);
            xlabel("Range (km)")
            ylabel("Percent Fuel Saving (%)");
            yline(0,"--",'HandleVisibility','off');
            hold on
        end
        title(strcat(aircraftNames(i), " Battery Energy Density = ",string(eBat(q)/(3600*0.8)),"Wh/kg"));
        legend("Location","southwest");
        saveas(figure(3*(q-1)+i),strcat("Figures\",aircraftNames(i),"\Battery Energy Density = ",string(eBat(q)/(3600*0.8))),"png");
    end
end
