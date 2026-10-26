import math as m
import numpy as np
L=50  ##link length
r=26   #platform center link
fi=30  #platform half angle
rx=r*m.cos(30*m.pi/180)
ry=r*m.sin(30*m.pi/180)
##parallel analyse
xp=76  #footprint x extention
yp=44  #footprint y extention
dx=0
dy=-15 #platform center cordination
yR=87.5 #radius of second manipulator
##first manipulator
px1=-72#-(r*m.cos(30*m.pi/180))   ##cordination of x position of end effictor of manipulator one
py1=18#-(r*m.sin(30*m.pi/180))        ##cordination of y position of end effictor of manipulator one

a=m.sqrt((px1**2+py1**2)/4)
h=m.sqrt(L**2-a**2)
alfa=m.atan2(h,a)*(180/m.pi);
beta=m.atan2(py1,px1)*(180/m.pi);
if beta<0:
    betaf=360+beta
    #theta1=(alfa+abs(beta))
    theta= betaf+alfa
else:
    thet1=alfa+abs(beta)
    if thet1<0:
        theta=thet1+360
    else:
        theta=thet1
#print(px1,py1)
print(alfa,beta,theta)
