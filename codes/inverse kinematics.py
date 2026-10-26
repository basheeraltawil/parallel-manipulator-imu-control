import math as m
import numpy as np
L=50  ##link length
r=26   #platform center link
fi=30  #platform half angle
rx=r*m.cos(30*m.pi/180)
ry=r*m.sin(30*m.pi/180)
px1=52#-(r*m.cos(30*m.pi/180))   ##cordination of x position of end effictor of manipulator one
py1=53#-(r*m.sin(30*m.pi/180))        ##cordination of y position of end effictor of manipulator one
a=m.sqrt((px1**2+py1**2)/4)
h=m.sqrt(L**2-a**2)
alfa=m.atan2(h,a)*(180/m.pi);
beta=m.atan2(py1,px1)*(180/m.pi);
if beta<0:
    theta1=(alfa+abs(beta))
    theta=360-theta1
else:
    theta=-alfa+abs(beta)

print(theta)
