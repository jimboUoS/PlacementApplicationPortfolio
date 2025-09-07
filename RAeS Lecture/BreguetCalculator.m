%Write code to automatically check calculations

%physical constants
g=9.81;
%aircraft params

%efficiencies
PSFC = [0.0000001113484452 0.00000007789322188 0.0000000790759823];

liftDragRatio=[13.5 14 16.18];

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
eBat=0.8*300*3600;
eFuel = 43.1*10^6;

%inputs
phi = 0.522995;
MTOWMultiplier = 2

%range calculations
massEnergy = MTOWMultiplier*aircraftMTOW - massPayload - massOperatingEmpty;
massFuelHybrid= (1-phi).*massEnergy.*eBat/(phi*eFuel+(1-phi)*eBat);
%massFuelConventional=aircraftMTOW-massPayload-massOperatingEmpty;

rangeConventional = ((etaProp.*liftDragRatio)./(g.*PSFC)).*log(aircraftMTOW./(massPayload+massOperatingEmpty))

rangeHybrid=(liftDragRatio./g).*(etaFuelChain*eFuel.*log((massEnergy+massPayload+massOperatingEmpty)./(massPayload+massOperatingEmpty+massEnergy-massFuelHybrid)) + etaBatChain*eBat*(massEnergy-massFuelHybrid)./(massPayload+massOperatingEmpty+massEnergy-massFuelHybrid))

massFuelConventional = (aircraftMTOW).*(1-exp(-g.*PSFC.*rangeHybrid/(liftDragRatio.*etaProp)));

percentFuelSaving = (1-(massFuelHybrid./massFuelConventional))*100

