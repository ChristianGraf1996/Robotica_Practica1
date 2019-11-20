import numpy as np
import math as m
from numpy.linalg import *


def reference_transformation_matrix(theta, d, alpha, a):
    res = [[m.cos(theta), -m.cos(alpha)*m.sin(theta), m.sin(alpha)*m.sin(theta), a*m.cos(theta)],
           [m.sin(theta), m.cos(alpha)*m.cos(theta), -m.sin(alpha)*m.cos(theta), a*m.sin(theta)],
           [0, m.sin(alpha), m.cos(alpha), d],
           [0, 0, 0, 1]]
    res = normalize_zeroes(res)
    return res


def normalize_zeroes(m):
    res = m
    for i in range(len(res)):
        for j in range(len(res[0])):
            if m[i][j] < 0.001 and m[i][j] > -0.001:
                res[i][j] = 0
            else:
                res[i][j] = m[i][j]
    return res


def multiply_matrix(matrix_array):
    res = matrix_array[0]
    for m in range(1, len(matrix_array)):
        res = np.dot(res, matrix_array[m])
    return res

def outputValues(m):
    s0 = []
    for i in range(len(m) - 1):
        s0.append(m[i][len(m) - 1])

    xyz = []
    for j in range(len(m)-1):
        for i in range(len(m)-1):
            if m[i][j] != 0:
                xyz.append(m[i][j]*i)

    return xyz,s0
 
def jacobian(a1,t1,t2,d):
    # arr containts [x,y,d]
    res =  [[( -a1*m.sin(t1) - d*m.cos(t1+t2)), d*m.cos(t1+t2), m.cos(t1+t2)],
            [ a1*m.sin(t1) + d*m.sin(t1+t2),  d*m.sin(t1+t2), m.sin(t1+t2)],
            [1,1,0]]
    res = normalize_zeroes(res)
    return res

dMin = 1
a1 = 1

def kinematics_inverse(x, y, phi):
    a = 1
    b = 2 * (y * m.cos(phi) - x * m.sin(phi))
    c = m.pow(x, 2) + m.pow(y, 2) - m.pow(a1, 2)
    d1 = (-b + m.sqrt( m.pow(b, 2) - (4 * a * c) ) )/2*a
    d2 = (-b - m.sqrt( m.pow(b, 2) - (4 * a * c) ) )/2*a
    resultSet = []
    resultSet.append(calculate_result(x, y, phi, d1))
    resultSet.append(calculate_result(x, y, phi, d2))
    return resultSet

def calculate_result(x, y, phi, d):
    c2 = (m.pow(x, 2) + m.pow(y, 2) - m.pow(a1, 2) - m.pow(d, 2)) / (2 * a1 * d)
    s2Set = []
    s2Set.append(m.sqrt(1 - m.pow(c2, 2)))
    s2Set.append( - (m.sqrt(1 - m.pow(c2, 2))))
    resultSet = []
    resultSet.append( [m.atan2(s2Set[0], c2), phi - m.atan2(s2Set[1], c2) - m.pi/2, d - dMin] )
    resultSet.append( [m.atan2(s2Set[1], c2), phi - m.atan2(s2Set[1], c2) - m.pi/2, d - dMin] )
    return resultSet

def calculate_inv(jacobian,x):
    invm = inv(jacobian)
    res = multiply_matrix([invm,x])
    return res

def calculate_pinv(jacobian,x):
    invm = pinv(jacobian)
    res = multiply_matrix([invm,x])
    return res

def calculate_tinv(jacobian,x):
    invm = np.transpose(jacobian)
    res = multiply_matrix([invm,x])
    return res
