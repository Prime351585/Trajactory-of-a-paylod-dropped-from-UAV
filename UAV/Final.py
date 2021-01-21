import os
import math
import numpy as np
import matplotlib.pyplot as plt
from math import radians, cos, sin, asin, sqrt
# for graphing ///////////////////////////////////////////////////////////////////////////////////
Y=[]                             # Actual Y coordinate Array
X=[]                             # Actual X coordinate Array
T=[]                             # Time Array
XPredFinal=[]                    # Predicted X coordinate Array
YPredFinal=[]                    # Predicted Y coordinate Array
Acc=[]                           # Accuracy Array

# For reading given file data /////////////////////////////////////////////////////////////////////
Workingdirectory=os.getcwd()
filepath=Workingdirectory+'\initial_waypoints.txt'
print(filepath)
initialwaypoints=open(filepath) #Enter the path of initialwaypoints.txt file

# initial Waypoints 
latitude=[]                      # Latitude Array Readed from file 
longitude=[]                     # Longitude Array Readed from file
VelocityX=[]                     # Velocity in X direction as readed from file
VelocityY=[]                     # Velocity in Y direction as readed from file

for x in initialwaypoints:
    c=x.split(" ")
    latitude.append(c[0])
    longitude.append(c[1])
    VelocityX.append(c[2])
    VelocityY.append(c[3])
initialwaypoints.close

# Final Waypoints
filepath=Workingdirectory+r"\final_waypoints.txt"
finalwaypoints=open(filepath)#Enter the path of finalwaypoints.txt file
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
# checking the waypoints////////////////////////////////////////////////////////////////////////////

W=[]                             # Wyapoint List
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

A=math.pi*0.05**2                # Surface Area for a Spherical payload of radius = 5 cm    
P=1.225                          # density of air
C=0.5                            # AIr drag corfficient
M=1.5                            # Mass of payload
Gravity=9.81                     # Gravity
t=0                              # initial time
Fdragx=0                         # initial Drag Force
Xi=0                             # initial X Position Of drone 
Yi=22                            # Hieght of Drone can be calculated 
Vw=2                             # Wind velocity can be altered from here

# Resultant velocity //////////////////////////////////////////////////////////////////////////////
theta=math.atan(Uy/Ux)           # velocity vector angle
U=sqrt((Ux)**(2)+(Uy)**2+2*math.cos(theta)*Ux*Uy)+Vw
Verticalvel=0                    # velocity of UAV in y axis
DelT=0.01                        # Small interval of time

# Main Calculation function ///////////////////////////////////////////////////////////////////////

def Trajectorycal(U,DelT,Xi,Yi,Verticalvel):
    
    for t in np.arange(0,120,DelT):   # Upto 1 Minute of dropping time,for More then 1 minute of dropping change 60 to desired time in seconds. 
        
        # drag force in horizontal direction
        Fdragx=(-1/2)*(P*((U-Vw)**2)*C*A)
        
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
        Error=abs(((minimumdistance-Xf)/minimumdistance))
        Accuracy=1-Error
        
        # FOR FURTHER GRAPHING AND CALCULATIONS
        X.append(Xf)
        Y.append(Yf)
        XPredFinal.append(Xf)
        YPredFinal.append(Yf)
        T.append(t)
        if Yf<0:
            Acc.append(Accuracy)
            break

#Calling Function  ////////////////////////////////////////////////////////////
Trajectorycal(U,DelT,Xi,Yi,Verticalvel)
print('Paylod dropped successfully','Paylod drop location',X[-1],'m','Drop time',T[-1],'sec','Accuracy =',Acc[0]*100,'%')

# FOR GRAPHING PATH OF PAYLOAD 
def Graphing(X,Y,LABEL):
    plt.plot(X,Y,label=LABEL)
    plt.title('Graph of Vertical Height vs Distance from the target')
    plt.xlabel('Distance of Target in m')
    plt.ylabel('Distance of Paylod in m')
    plt.legend(loc='upper right')
Graphing(X,Y,'ACTUAL PATH')

## FOR CALCULATION OF HEIGHT OF ~99% ACCURACY /////////////////////////////////
y=100
i=0
Ypred=0
while True:
    Trajectorycal(U,DelT,Xi,y,Verticalvel)
    if Acc[i]>0.5:
        y+=0.5
        Trajectorycal(U,DelT,Xi,y,Verticalvel)
    elif Acc[i]<=0.5:
        y-=0.5
        Trajectorycal(U,DelT,Xi,y,Verticalvel)
    if max(Acc)<0.999 and max(Acc)>0.980:      # This range can be altered for an Accuracy less then 80%(If you Want to play with the code by increasing the Velocity of UAV)
        Ypred=y
        print('Warning With Current Height only an Accuracy of',Acc[0]*100,'% can be obtained')
        print('For an accuracy of',max(Acc)*100,'% it is Reccomended to Lock on UAV to an Altitude of',Ypred,'m',',Drop Time Will be =',T[-1] ,'sec')
        break
    if i>10000:
        print("Error Either Wind velocity is greater then UAV's Velocity Or Veocity of the UAV is too High ")
        break
    i+=1
    
# FOR GRAPHING PREDICTED PATH ///////////////////////////////////////////////////

XPredFinal.clear()
YPredFinal.clear()
X.clear()
Y.clear()
T.clear()
Trajectorycal(U,DelT,Xi,Ypred,Verticalvel)
Graphing(XPredFinal,YPredFinal,"PREDICTED PATH")

# For printing the predicted path ///////////////////////////////////////////////


plt.show() # It is used in the end as to plot the both  the graph on one

#Created by Harsh Maurya with 💖 ............ :)🌈
