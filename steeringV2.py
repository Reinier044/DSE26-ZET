import math as m
import numpy as np

turn='left'
V_in=2
alpha=50
l=7.59
d=16.906

R=d*m.tan(m.pi/2-m.radians(alpha))

if R==0:
    omega_ac=V_in/(l/2)
else:
    omega_ac=V_in/R

V_L=omega_ac*(-l/2+d*m.tan(m.pi/2-m.radians(alpha)))
V_R=omega_ac*(l/2+d*m.tan(m.pi/2-m.radians(alpha)))
V_N=omega_ac*d*m.sqrt(1+m.tan(m.pi/2-m.radians(alpha))**2)


alpha_t=alpha-75
g=1.88
h=3.65
l_b=3.095
l_f=2.865
alpha_l=30
alpha_r_max=20.71
R_ev=g/m.cos(m.pi/2-m.radians(alpha_t))
omega_ev=V_N/R_ev
V_lb=omega_ev*(g*m.tan(m.pi/2-m.radians(alpha_t))-l_b/2)
V_rb=omega_ev*(g*m.tan(m.pi/2-m.radians(alpha_t))+l_b/2)
V_lf=omega_ev*h/m.cos(m.pi/2-m.radians(alpha_l))
V_rf=omega_ev*m.sqrt((h*m.tan(m.pi/2-m.radians(alpha_l))+l_f)**2+h**2)
alpha_r=90-m.degrees(m.acos(h/m.sqrt((h*m.tan(m.pi/2-m.radians(alpha_l))+l_f)**2+h**2)))


print("Turning radius from middle of MLG "+str(R))
print(V_L,V_N,V_R)
print("External vehicle speed dist.")
print(V_lf, V_rf)
print(V_lb, V_rb)
print("Turning angle of right wheel "+str(alpha_r))
