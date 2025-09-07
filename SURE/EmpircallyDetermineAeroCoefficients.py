################################################################################################
# Log data must be in the following format:
# timestamp(ms)	CTUN.ThO	ARSP[0].Airspeed	ARSP[0].DiffPress	ARSP[0].Temp    ATT.Pitch	
# ATT.Roll	    RCOU.C3	    RCOU.C11	        AOA.AOA	            IMU[0].AccX	    IMU[0].AccZ
################################################################################################

#the following calculations assume horizontal flight where the tilt rotors are in-line with the aircraft x axis
#please refer to the checklist to ensure that the log data input into this program is valid

#imports
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
#import seaborn as sns

#allow pandas to print all columns of a dataframe
pd.set_option('display.max_columns', None)

#physical constants
g=9.81

#aircraft parameters
meanChord = 0.320
wingArea = 0.752
aircraftMass = 9.2

#definition of log and air properties data frames
airArr = [i.split() for i in """150 0.9432 0.6562 1.437 0.01393 1.045 0.7076 2.323
200 0.9882 0.7012 1.409 0.01797 1.346 0.7406 1.742
250 1.005 0.7176 1.400 0.02187 1.614 0.7414 1.394
260 1.006 0.7185 1.399 0.02264 1.664 0.7394 1.340
280 1.006 0.7194 1.399 0.02415 1.763 0.7345 1.244
293.15 1.007 0.7196 1.399 0.02514 1.825 0.7309 1.189
300 1.007 0.7197 1.399 0.02565 1.857 0.7290 1.161
320 1.007 0.7199 1.399 0.02712 1.949 0.7236 1.089
340 1.007 0.7203 1.398 0.02858 2.038 0.7185 1.025
350 1.008 0.7207 1.398 0.02930 2.082 0.7161 0.996
400 1.012 0.7246 1.396 0.03283 2.292 0.7062 0.871
450 1.019 0.7316 1.392 0.03625 2.489 0.6996 0.774
500 1.028 0.7409 1.387 0.03955 2.676 0.6956 0.697
600 1.051 0.7636 1.376 0.04582 3.024 0.6935 0.581
700 1.075 0.7880 1.364 0.05168 3.344 0.6956 0.498
800 1.099 0.8117 1.354 0.05716 3.641 0.6999 0.436
1000 1.140 0.8532 1.336 0.06706 4.180 0.7108 0.348
1200 1.173 0.8863 1.324 0.07576 4.662 0.7220 0.290
1400 1.199 0.9122 1.315 0.08347 5.098 0.7325 0.249
1600 1.220 0.9326 1.308 0.09042 5.497 0.7415 0.218
1800 1.236 0.9489 1.302 0.09682 5.865 0.7486 0.194
2000 1.249 0.9621 1.298 0.1029 6.204 0.7531 0.174
2500 1.274 0.9866 1.291 0.1182 6.953 0.7491 0.139
3000 1.291 1.004 1.286 0.1364 7.582 0.7177 0.116 """.split("\n")]
airArr = np.array(airArr,dtype="float32")

airProperties = pd.DataFrame(data=airArr)
headers = "T Cp Cv Gamma k mu*10^5 Pr rho".split()
airProperties.columns=headers

airProperties.set_index("T", inplace=True)

logFileName = "CompFinalFlight.csv"
".csv"
logData = pd.read_csv(logFileName)

extendedAirLookup = airProperties.copy()
extendedLogData = logData.copy()

#definition of thrust data
thrustLookup = np.array([[50.0, 55.0, 60.0, 65.0, 75.0, 85.0, 100.0],[2*x*g/1000 for x in [2683.0,3075.0,3444.0,3802.0,4609.0,5459.0,6762.0]]])

#PWM bounds
#give these a second look over

# #bounds on the angle of the tilt rotors, 
# vectorAngleLeftPWMBounds = (930,1942)
# vectorAngleRightPWMBounds = (1965,1036)
# #defined wrt to the aircraft x axis
# #in degrees
# vectorAngleBounds = (90,0)

#bounds on the angle of the elevator
#positive angle means elevator deflects upwards causing pitch up
elevatorPWMBounds = (1500,1900)
#in degrees
elevatorAngleBounds = (0,25)

#bounds on the angle of the ailerons
#the ailerons are symmetric therefore read in values for one aileron

aileronPWMBounds = (988,2011)
#in degrees
aileronAngleBounds = (-30,30)


#function definitions
def getAirProperties(T):
    global extendedAirLookup
    if T>max(airProperties.index) or T<min(airProperties.index):
        # if T<(min(airProperties.index)):
        #     print("Error! Enter temperature greater than or equal to 150K.\n")
        # elif T>max(airProperties.index):
        #     print("Error! Enter temperature less than 3000K.\n")
        # else:
        #     print("Error! Invalid input.\n")
        return -1
    #return value is tuple in format (density,viscosity)
    if T in airProperties.index:
        return 0
        #print(airProperties.loc[T])
    else:
    #use binary search to determine bounds
        T_high = (max(airProperties.index))
        T_low = min(airProperties.index)
        midPoint = airProperties.index[len(airProperties.index)//2]
        
        while(midPoint != T_low and midPoint != T_high):
            if midPoint < T:
                T_low=midPoint
            elif midPoint > T:
                T_high=midPoint
            midPoint = airProperties.index[(np.where(airProperties.index==T_low)[0][0] + np.where(airProperties.index==T_high)[0][0])//2]
        interpFactor = (T-T_low)/(T_high-T_low)
        Values_low = airProperties.loc[T_low]
        Values_high = airProperties.loc[T_high]
        Values_result = Values_low + interpFactor*(Values_high-Values_low)
        #print(Values_result.rename(T).to_frame().T)
        extendedAirLookup = pd.concat([extendedAirLookup, Values_result.rename(T).to_frame().T]).sort_index()
        return 0

def getAirDensity(T):
    if T not in extendedAirLookup.index:
        getAirProperties(T)
    return extendedAirLookup.loc[T]["rho"]

def getAirViscosity(T):
    if T not in extendedAirLookup.index:
        getAirProperties(T)
    return extendedAirLookup.loc[T]["mu*10^5"]*(10**-5)

#manipulate log data to convert temperature to kelvin
extendedLogData["TempKelvin"] =extendedLogData["ARSP[0].Temp"] + 273.15

# #use the timestamp since boot as a unique identifier
# extendedLogData.set_index("timestamp(ms)",inplace=True)

#temperature has been converted to Kelvin, drop original celsius reading
extendedLogData.drop("ARSP[0].Temp",axis=1,inplace=True)

#rename columns for easier reading
extendedLogData.rename(columns={"CTUN.ThO": "Throttle", "ARSP[0].Airspeed": "Airspeed", "AOA.AOA": "AngleOfAttack","ARSP[0].DiffPress": "DynamicPressure", "ATT.Roll":"Roll", "ATT.Pitch":"Pitch", "RCOU.C3":"AileronPWM", "RCOU.C11":"ElevatorPWM", "IMU[0].AccX":"AccelerationX", "IMU[0].AccZ":"AccelerationZ"}, errors="raise", inplace=True)

#add temporary viscosity and density columns to calculate reynolds and dynamic pressure
extendedLogData["AirDensity"] = extendedLogData["TempKelvin"].apply(getAirDensity)
extendedLogData["AirViscosity"] = extendedLogData["TempKelvin"].apply(getAirViscosity)

#calculate reynolds
extendedLogData["Reynolds"]=(extendedLogData["AirDensity"]*extendedLogData["Airspeed"]*meanChord)/extendedLogData["AirViscosity"]

# #calculate dynamic pressure
# extendedLogData["DynamicPressure"]=0.5*extendedLogData["AirDensity"]*extendedLogData["Airspeed"]**2

#drop viscosity and density columns
extendedLogData.drop("AirDensity",axis=1,inplace=True)
extendedLogData.drop("AirViscosity",axis=1,inplace=True)

#calculate control surface deflections
extendedLogData["ElevatorAngle"] = np.interp(extendedLogData["ElevatorPWM"],elevatorPWMBounds,elevatorAngleBounds)
extendedLogData["AileronAngle"] = np.interp(extendedLogData["AileronPWM"],aileronPWMBounds,aileronAngleBounds)

print(extendedLogData.head())

#bounds on exclusion of data points
reynoldsExclusionLower = 250000
reynoldsExclusionUpper = 350000
elevatorExclusionAbs = 1
aileronExclusionAbs = 1
angleOfAttackExclusionAbs = 8

lambda x: False if (extendedLogData["Reynolds"]<reynoldsExclusionLower or abs(extendedLogData["ElevatorAngle"]) > elevatorExclusionAbs or abs(extendedLogData["AileronAngle"]) > aileronExclusionAbs) else True

#exclude data points that meet the exclusion criteria
extendedLogData.drop(extendedLogData[extendedLogData["Reynolds"]<reynoldsExclusionLower].index, inplace=True)
extendedLogData.drop(extendedLogData[extendedLogData["Reynolds"]>reynoldsExclusionUpper].index, inplace=True)
extendedLogData.drop(extendedLogData[abs(extendedLogData["ElevatorAngle"]) > elevatorExclusionAbs].index, inplace=True)
extendedLogData.drop(extendedLogData[abs(extendedLogData["AileronAngle"]) > aileronExclusionAbs].index, inplace=True)
extendedLogData.drop(extendedLogData[abs(extendedLogData["AngleOfAttack"]) > angleOfAttackExclusionAbs].index, inplace=True)

print("********************\nAFTER EXCLUSION\n********************")
print(extendedLogData.head())

#solve for lift and drag, then turn into coefficients and create polar

print("****************************************\nBEGIN SOLVING FOR LIFT AND DRAG\n****************************************")

polarDataframe = pd.DataFrame(columns=["AngleOfAttack","Lift","Drag","CoefficientOfLift","CoefficientOfDrag"])

for rowIndex,row in extendedLogData.iterrows():
    alphaRotationMatrix = np.array([[np.sin(np.deg2rad(row["AngleOfAttack"])),-np.cos(np.deg2rad(row["AngleOfAttack"]))],[np.cos(np.deg2rad(row["AngleOfAttack"])),np.sin(np.deg2rad(row["AngleOfAttack"]))]])
    thrust = np.interp(row["Throttle"],*thrustLookup)
    #resultVector = np.array([[aircraftMass*row["AccelerationX"]-thrust+aircraftMass*g*np.sin(np.deg2rad(row["Pitch"]))],[aircraftMass*row["AccelerationZ"]+aircraftMass*g*np.cos(np.deg2rad(row["Pitch"]))*np.cos(np.deg2rad(row["Roll"]))]])
    #resultVector = np.array([[aircraftMass*row["AccelerationX"]-thrust],[aircraftMass*row["AccelerationZ"]]])
    resultVector = np.array([[aircraftMass*row["AccelerationX"]-thrust+2*aircraftMass*g*np.sin(np.deg2rad(row["Pitch"]))],[-aircraftMass*row["AccelerationZ"]]])
    liftDragVector = np.linalg.solve(alphaRotationMatrix,resultVector)

    polarRow = pd.Series(np.concatenate(([row["AngleOfAttack"]], *(liftDragVector.T), *(liftDragVector.T/(row["DynamicPressure"]*wingArea)))),name=rowIndex,index=["AngleOfAttack","Lift","Drag","CoefficientOfLift","CoefficientOfDrag"])
    #print(polarRow)
    polarDataframe.loc[rowIndex]=polarRow

polarDataframe.sort_values("AngleOfAttack",inplace=True)

print(polarDataframe.head())

alphaLinSpace = np.linspace(-angleOfAttackExclusionAbs,angleOfAttackExclusionAbs,2)
coefficientLiftLinSpace = np.linspace(-5,5,20)
coefficientLiftSquaredLinSpace = np.linspace(0,5,20)
fig, axs = plt.subplots(1,3)

fig.suptitle(f"Empirical Polar Calculations\n Currrent Log: {logFileName}")

liftRegression = np.polyfit(polarDataframe["AngleOfAttack"],polarDataframe["CoefficientOfLift"],deg=1)
axs[0].scatter(polarDataframe["AngleOfAttack"],polarDataframe["CoefficientOfLift"])
axs[0].plot(alphaLinSpace, liftRegression[1] + liftRegression[0] * alphaLinSpace)
axs[0].set_title(r"$C_L = $" + str(round(liftRegression[0],4)) +r"$\alpha + $"+str(round(liftRegression[1],4)))
axs[0].set_ylabel(r"$C_L$")
axs[0].set_xlabel(r"$\alpha$ / $ ^\circ $")

dragRegression = np.polyfit(polarDataframe["CoefficientOfLift"],polarDataframe["CoefficientOfDrag"],deg=2)
axs[1].scatter(polarDataframe["CoefficientOfLift"],polarDataframe["CoefficientOfDrag"])
axs[1].plot(coefficientLiftLinSpace, dragRegression[2] + dragRegression[1] * coefficientLiftLinSpace + dragRegression[0]*coefficientLiftLinSpace**2)
axs[1].set_title(r"$C_D = $"+str(round(dragRegression[0],4))+r"${C_L}^2+$"+str(round(dragRegression[1],4))+r"$C_L+$"+str(round(dragRegression[2],4)))
axs[1].set_ylabel(r"$C_D$")
axs[1].set_xlabel(r"$C_L$")

kRegression = np.polyfit(np.square(polarDataframe["CoefficientOfLift"]),polarDataframe["CoefficientOfDrag"],deg=1)
axs[2].scatter(np.square(polarDataframe["CoefficientOfLift"]),polarDataframe["CoefficientOfDrag"])
axs[2].plot(coefficientLiftSquaredLinSpace, kRegression[0]*(coefficientLiftSquaredLinSpace)+kRegression[1])
axs[2].set_title(r"$C_D = $"+str(round(kRegression[1],4))+r"$ + $"+str(round(kRegression[0],4))+r"${C_L}^2$")
axs[2].set_ylabel(r"$C_D$")
axs[2].set_xlabel(r"${C_L}^2$")


print(f"Number of datapoints: {len(extendedLogData)}")
plt.show()


