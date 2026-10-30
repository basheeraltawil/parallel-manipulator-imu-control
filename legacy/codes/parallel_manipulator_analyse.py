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
px1=-(xp-13)+dx#-(r*m.cos(30*m.pi/180))   ##cordination of x position of end effictor of manipulator one
py1=(yp+7.5)+dy#-(r*m.sin(30*m.pi/180))        ##cordination of y position of end effictor of manipulator one

a=m.sqrt((px1**2+py1**2)/4)
h=m.sqrt(L**2-a**2)
alfa=m.atan2(h,a)*(180/m.pi);
beta=m.atan2(py1,px1)*(180/m.pi);
if beta<0:
    theta1=(alfa+abs(beta))
    theta=360-theta1
else:
    thet1=-alfa+abs(beta)
    if thet1<0:
        theta=thet1+360
    else:
        theta=thet1
#print(px1,py1)
print(px1,py1,alfa,beta,theta)
##second manipulator
px2=(dx-13)#-(r*m.cos(30*m.pi/180))   ##cordination of x position of end effictor of manipulator one
py2=-(yR-7.5)+dy#-(r*m.sin(30*m.pi/180))        ##cordination of y position of end effictor of manipulator one
a2=m.sqrt((px2**2+py2**2)/4)
h2=m.sqrt(L**2-a2**2)
alfa2=m.atan2(h2,a2)*(180/m.pi);
beta2=m.atan2(py2,px2)*(180/m.pi);
if beta2<0:
    theta22=(alfa2+abs(beta2))
    theta2=360-theta22
else:
    thet2=-alfa2+abs(beta2)
    if thet2<0:
        theta2=thet2+360
    else:
        theta2=thet2
print(px2,py2,alfa2,beta2,theta2)
##third anipulator
px3=xp+dx#-(r*m.cos(30*m.pi/180))   ##cordination of x position of end effictor of manipulator one
py3=(yp-15)+dy#-(r*m.sin(30*m.pi/180))        ##cordination of y position of end effictor of manipulator one
a3=m.sqrt((px3**2+py3**2)/4)
h3=m.sqrt(L**2-a3**2)
alfa3=m.atan2(h3,a3)*(180/m.pi);
beta3=m.atan2(py3,px3)*(180/m.pi);
if beta3<0:
    theta33=(alfa3+abs(beta3))
    theta3=360-theta33
else:
    thet3=-alfa3+abs(beta3)
    if thet3<0:
        theta3=thet3+360
    else:
        theta3=thet3

print(px3,py3,alfa3,beta3,theta3)
