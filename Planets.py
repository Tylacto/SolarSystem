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
    method = 1
    """mass is set to 1. to ensure it is a float and non-integer."""
