import copy
import matplotlib.pyplot as plt

from Planets import Planet

Earth = Planet([0,0,0],[0,0,0],[0,0,0],"Earth",5.972e24,1)
Satellite = Planet([7.271e6,0,0],[0,7401,0],[0,0,0],"Satellite",1e3,1)

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
deltaT = 5

x = []
y = [] 
data = []
 
while time < 5e4:
    time += deltaT
    step(massList,positionList,deltaT)
    print("Time: %6.3f, %s"%(time,Satellite))
    item=[time, copy.deepcopy(Satellite) ]
    data.append(item)
    y.append(Satellite.position[0])
    x.append(Satellite.position[1])

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