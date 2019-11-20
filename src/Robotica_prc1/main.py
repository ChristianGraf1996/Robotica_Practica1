import math as m
from prc3 import *
from numpy.linalg import *
import matplotlib.pyplot as plt

# m1 = reference_transformation_matrix(m.pi, 0, 0, 1) # a1 = 1
# m2 = reference_transformation_matrix(m.pi/2, 0, m.pi/2, 0)
# m3 = reference_transformation_matrix(0, 1.5, 0, 0) # dmin = 2 
# mArr = []
# mArr.append(m1)
# mArr.append(m2)
# mArr.append(m3)
# res = multiply_matrix(mArr)
# print(res)

jacobM = jacobian(1,0,m.pi,1.5)
qvec = [m.pi/90,0,-1/100]
x = multiply_matrix([jacobM,qvec])
#inv = inv(jacobM)
print('jacobiana: ' , jacobM)
print('vector q : ', qvec)
print('x: ', x)
#inv = calculate_inv(jacobM,x)
#print('con inversa: ', inv)
pinv = calculate_pinv(jacobM,x)
print('con pseudo-inversa: ', pinv)
tinv = calculate_tinv(jacobM,x)
print('con transpuesta: ', tinv)
# plt.plot(inv,inv)
# plt.plot(pinv,qvec)
# plt.show()
# test1 = kinematics_inverse(-2.5, 0, 3*m.pi/2)
# print(test1)