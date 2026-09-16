import numpy as np
# arctan
Adx = -0.875
Ady = -0.486
Bdx = 1
Bdy = 0
Eq1 = Bdx*Ady - Bdy*Adx
Eq2 = Bdx*Adx + Bdy*Ady
theta = np.arctan2(Eq1, Eq2)
print(theta*180/np.pi)
print("Rotation Matrix")
R = np.array([[np.cos(theta), -np.sin(theta), 0],
               [np.sin(theta), np.cos(theta), 0],
               [0, 0, 1]])
theta_2 = 60
theta_2 = theta_2*np.pi/180
print(theta_2)
R2 = np.array([[1, 0, 0],
               [0, np.cos(theta_2), -np.sin(theta_2)],
               [0, np.sin(theta_2), np.cos(theta_2)]])

print(R@R2)

