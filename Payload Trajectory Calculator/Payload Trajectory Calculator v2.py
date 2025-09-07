import math, csv, numpy
import matplotlib.pyplot as plt
from scipy.stats import linregress

#MODEL 2
#This model assumes the parachute-payload system is horizontal at the moment of deployment
#assumes taught string

velocity = [25,0]
# for use in the graph
#all units are base SI units

#lists used for plots
velocityHorizontalPoints = []
velocityVerticalPoints = []
#velocityLogHorizontalPoints = []
#velocityLogVerticalPoints = []
displacementHorizontalPoints =[]
displacementVerticalPoints = []

timePoints = []
displacement = [0,0]
initialHeight = 25
mass = 2
parachuteDiameter = 0.40
C_dparachute = 0.75
deltaT = 0.001
timeElapsed = 0

def calculateDragMagnitude():
    #assumes parachute causes all the drag
    parachuteArea = (math.pi/4)*parachuteDiameter**2
    return 0.6125*C_dparachute*parachuteArea*vectorMagnitude(velocity)**2

def vectorMagnitude(vct):
    #calculates vector magnitude using pythagoras' theorem
    return math.sqrt((vct[0]**2) + (vct[1]**2))

def vectorAngle(vct):
    #calculates the angle between a given vector and the vertical
    #the ratio of x/y for velocity is the same as for drag
    #returns angle in radians
    #works off of the dot product between the given vector and the [0,1] vector ==> cos(theta)= (y component of given vector)/(magnitude of given vector)
    return math.acos(vct[1]/vectorMagnitude(vct))

#def deltaVS(vct):
   #for each vector component, multiply by deltaT to get a small change in velocity or displacement
#    return [deltaT*i for i in vct]


#def acceleration():
#    return [(-calculateDragMagnitude()*math.sin(vectorAngle(velocity)))/mass, 9.81 - (calculateDragMagnitude()*math.cos(vectorAngle(velocity)))/mass]

#a_0=[-calculateDragMagnitude()/mass,9.81]

acceleration = [-calculateDragMagnitude()/mass,9.81]
while displacement[1]<initialHeight:
    velocity[0] += acceleration[0]*deltaT
    velocity[1] += acceleration[1]*deltaT
    displacement[0] += velocity[0]*deltaT
    displacement[1] += velocity[1]*deltaT
    velocityHorizontalPoints.append(velocity[0])
    velocityVerticalPoints.append(velocity[1])
    displacementHorizontalPoints.append(displacement[0])
    displacementVerticalPoints.append(displacement[1])
    #velocityLogHorizontalPoints.append(math.log(velocity[0]))
    #velocityLogVerticalPoints.append(math.log(velocity[1]))

    acceleration = [(-calculateDragMagnitude()*math.sin(vectorAngle(velocity)))/mass, 9.81 - (calculateDragMagnitude()*math.cos(vectorAngle(velocity)))/mass]
    timeElapsed += deltaT
    timePoints.append(timeElapsed)


'''
with open("velocityHorizontalPoints.csv", 'w', newline = '') as vHFile:
    vHWriter = csv.writer(vHFile)
    vHWriter.writerows([timePoints,velocityLogHorizontalPoints])
vHFile.close()

with open("velocityVerticalPoints.csv", 'w', newline = '') as vVFile:
    vVWriter = csv.writer(vVFile)
    vVWriter.writerows([timePoints,velocityLogVerticalPoints])
vVFile.close()

print(f"Log Horizontal Velocity Against Time Regression Calc: {linregress(timePoints, velocityLogHorizontalPoints)}")
print(f"Log Vertical Velocity Against Time Regression Calc: {linregress(timePoints, velocityLogVerticalPoints)}")
'''

print(f"Horizontal Displacement: {displacement[0]} m")
print(f"Vertical Overshoot : {((displacement[1] - initialHeight)/initialHeight)*100} %")
print(f"Time Elapsed: {timeElapsed} s")

#plotting code, ignore for implementation
velocityHorizontalFigure = plt.figure(1)
plt.plot(timePoints, velocityHorizontalPoints)
plt.xlabel("Time / s")
plt.ylabel("Velocity / ms^-1")
plt.title("Numerical Solution for Horizontal Velocity")

velocityVerticalFigure = plt.figure(2)
plt.plot(timePoints, velocityVerticalPoints)
plt.xlabel("Time / s")
plt.ylabel("Velocity / ms^-1")
plt.title("Numerical Solution for Vertical Velocity")

displacementHorizontalFigure = plt.figure(3)
plt.plot(timePoints, displacementHorizontalPoints)
plt.xlabel("Time / s")
plt.ylabel("Displacement / m")
plt.title("Numerical Solution for Horizontal Displacement")

displacementVerticalFigure = plt.figure(4)
plt.plot(timePoints, displacementVerticalPoints)
plt.xlabel("Time / s")
plt.ylabel("Displacement / m")
plt.title("Numerical Solution for Vertical Displacement")


heightPoints = []

for i in range(len(displacementVerticalPoints)):
    heightPoints.append(25-displacementVerticalPoints[i])
displacementTrajectoryFigure = plt.figure(5)
plt.plot(displacementHorizontalPoints, heightPoints)
plt.xlabel("Distance / m")
plt.ylabel("Height / m")
plt.title("Numerical Solution for Payload Trajectory")

plt.show()
