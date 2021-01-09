import math
import numpy as np
import matplotlib.pyplot as plt
from math import radians, cos, sin, asin, sqrt
# for graphing ///////////////////////////////////////////////////////////////////////////////////
Y=[]
X=[]
T=[]
# for reading given file data /////////////////////////////////////////////////////////////////////
initialwaypoints=open(r"initial_waypoints.txt") #Enter the path of initialwaypoints.txt file
# initial Waypoints 
latitude=[]
longitude=[]
VelocityX=[]
VelocityY=[]
for x in initialwaypoints:
    c=x.split(" ")
    latitude.append(c[0])
    longitude.append(c[1])
    VelocityX.append(c[2])
    VelocityY.append(c[3])
initialwaypoints.close
# Final Waypoints
finalwaypoints=open(r"final_waypoints.txt")#Enter the path of finalwaypoints.txt file
for x in finalwaypoints:
    c=x.split(" ")
    latitude.append(c[0])
    longitude.append(c[1])
FINALWAYPOINT=[latitude[-1],longitude[-1]]
finalwaypoints.close
# HAVERSINE FUNCTION ////////////////////////////////////////////////////////////////////////////////
def haversine(lon1, lat1, lon2, lat2):
    """
    Calculate the great circle distance between two points 
    on the earth (specified in decimal degrees)
    """
    # convert decimal degrees to radians 
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])
    # haversine formula 
    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a)) 
    r = 6371000                  # Radius of earth in kilometers. Use 3956 for miles
    return c * r
print('TESTING',latitude)
# checking the waypoints////////////////////////////////////////////////////////////////////////////
W=[]
print('checking for Waypoints')
for j in range(0,len(latitude)-1):
    W.append(haversine(float(longitude[j]),float(latitude[j]),float(longitude[-1]),float(latitude[-1])))
    print('Distance of final Waypoint from Waypoint',j+1,'=',W[j],'m')
minimumdistance=min(W)           # Finding Minimum Distance
pos=W.index(minimumdistance)     # Finding index of Minimum distance
print('Minimum Distance Found is between Final Waypoint and Waypoint',pos+1,'= ',minimumdistance,'m')
Ux=float(VelocityX[pos])         # x velocity of drone
Uy=float(VelocityY[pos])         # y velocity of drone
print('Velocity of UAV in x direction=',Ux)
print('Velocity of UAV in y direction =',Uy)

A=math.pi*0.05**2                # for radius = 5 cm    
P=1.225                          # density of air
C=0.5                            # AIr drag corfficient
M=1.5                            # Mass of payload
Gravity=9.81                     # Gravity
t=0
Fdragx=0
Xi=0
Yi=22                            # Hieght of Drone can be calculated 
Vw=2                             # Wind velocity can be altered from here
# Resultant velocity ////////////////////////////////////////////////////////////////////
theta=math.atan(Uy/Ux)#//////// velocity vector angle
U=sqrt((Ux)**(2)+(Uy)**2+2*math.cos(theta)*Ux*Uy)+Vw
Verticalvel=0
DelT=0.01                        # Small interval of time
for t in np.arange(0,60,DelT):   # For 1 Minute
    # drag force in horizontal direction
    Fdragx=(-1/2)*(P*(U**2)*C*A)
    # initial acceleration in horizontal direction
    Aix=Fdragx/M
    # velocity in horizontal direction
    Vx=U+Aix*DelT
    # distance in horizontal direction
    Xf=Xi+U*DelT+(1/2)*Aix*((DelT)**2)
    U=Vx
    Xi=Xf
    # drag force in vertical direction
    Fdragy=(1/2)*(P*(Verticalvel**2)*C*A)
    # initial acceleration in vertical direction
    Aiy=(Fdragy/M)-Gravity
    # velocity in horizontal direction
    Vy=Verticalvel+Aiy*DelT
    # distance in horizontal direction
    Yf=Yi+Verticalvel*DelT+(1/2)*Aiy*((DelT)**2)
    Verticalvel=Vy
    Yi=Yf
    # FOR GRAPH
    X.append(Xf)
    Y.append(Yf)
    T.append(t)
    Error=abs(((minimumdistance-Xf)/minimumdistance))
    Accuracy=1-Error
    if Yf<0:
        print('Paylod dropped successfully','Paylod drop location',Xf,'m','Drop time',t,'sec','Accuracy =',Accuracy*100,'%')
        break
# FOR GRAPH
plt.plot(X,Y)
plt.title('Graph of Vertical Height vs Distance from the target')
plt.xlabel('Distance of Target in m')
plt.ylabel('Distance of Paylod in m')
plt.show()
# /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
