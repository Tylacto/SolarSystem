import copy
import matplotlib.pyplot as plt
import numpy as np
import math

class Planet:
    """Class of methods including the Force to iterate over and the acceleration. Setting method = 1 uses Euler-Cromer, and setting method = 2 uses Euler-Forward."""
    
    def __init__(self, initialPosition, initialVelocity, initialAcceleration, Name, mass, method):
        
        if len(initialPosition) != 3:
            raise ValueError("The initial position array must be of length 3")
        self.position = np.array(initialPosition) * 1. #multiplying by 1. to turn array members into floats so += works with numpy
        
        if len(initialVelocity) != 3:
            raise ValueError("The initial velocity array must be of length 3")
        self.velocity = np.array(initialVelocity) * 1.
        
        if len(initialAcceleration) != 3:
            raise ValueError("The initial acceleration array must be of length 3")
        self.acceleration = np.array(initialAcceleration) * 1.
        
        self.Name = Name
        self.mass = mass

        self.setMethod(method)
    
    def __repr__(self):
        return 'Particle: %10s, Mass: %.5e, Position: %s, Velocity: %s, Acceleration:%s'%(self.Name,self.mass,self.position, self.velocity,self.acceleration)

    def setMethod(self,method):
        self.updateMethod = self.EulerCromer
        if method == 2:
            self.updateMethod = self.EulerForward

    def update(self, deltaT):
        """Implements the Given Method to update the position and velocity of a particle for a given time step"""
        self.updateMethod(deltaT)
        
    def EulerCromer(self, deltaT):
        self.velocity += (self.acceleration * deltaT)
        self.position += (self.velocity * deltaT)
        
    def EulerForward(self, deltaT):
        self.position += (self.velocity * deltaT)
        self.velocity += (self.acceleration * deltaT)

    def GForce(self,massList,positionList):
        """Refers to the force of body 2 on body 1"""
        G = 6.67408e-11
        force = 0
        for i in range(0,len(massList)):
            if positionList[i].all == self.position.all:
                continue
            distanceVector = positionList[i] - self.position
            distance = math.sqrt(distanceVector.dot(distanceVector))
            force = force + ((G * self.mass * massList[i])/(distance*distance*distance))*distanceVector
        return force

    def accelerationUpdater(self,massList,positionList):
        acceleration = self.GForce(massList,positionList)/(self.mass)
        return acceleration

    position = np.array([0,0,0])
    velocity = np.array([0,0,0])
    acceleration = np.array([0,0,0])
    name = "default"
    mass = 1.
    method = 2
    """mass is set to 1. to ensure it is a float and non-integer."""

G = 6.67408e-11
earthMass = 5.97237e24
earthRadius = 63710*1e3
satPosition = earthRadius+(35786*1e3)
satVelocity = math.sqrt(G*earthMass/(satPosition))

Earth = Planet([0,0,0],[0,10,0],[0,0,0],"Earth",earthMass,2)
Satellite = Planet([satPosition,0,0],[0,satVelocity,0],[0,0,0],"Satellite",100.,2)

objectsList = [Earth,Satellite]
positionList = []
massList = []


def positionListUpdate(objectsList):
    positionList.clear()
    for i in range(0,len(objectsList)):
        positionList.append(objectsList[i].position)

def massListUpdate(objectsList):
    massList.clear()
    for i in range(0,len(objectsList)):
        massList.append(objectsList[i].mass)

def step(massList,positionList,DeltaT):
    Earth.acceleration = Earth.accelerationUpdater(massList,positionList)
    Satellite.acceleration = Satellite.accelerationUpdater(massList,positionList)
    Earth.update(DeltaT)
    Satellite.update(DeltaT)
    positionListUpdate(objectsList)

positionListUpdate(objectsList)
massListUpdate(objectsList)

time = 0
deltaT = 6

x = []
y = [] 
data = []
 
while time < 1.2e6:
    time += deltaT
    step(massList,positionList,deltaT)
    item=[time, copy.deepcopy(Earth), copy.deepcopy(Satellite) ]
    data.append(item)
    y.append(Satellite.position[1])
    x.append(Satellite.position[0])
np.save("TwoBodyTest", data)

plt.plot(x,y,'r-',label='orbit')
plt.xlabel('x')
plt.ylabel('y')
plt.legend()
plt.show()

#print(massList)
#print(positionList)
#print(max(x))
#print(max(y))
#print(len(x))