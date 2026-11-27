original  = {"f": 1}

def f(o):
    o["f"] += 1
    return o

print(original, f(original))

print((1,2) + (3,4))

import numpy as np
A = np.array([[4, 7], 
              [2, 6]])

# Compute the inverse
A_inv = np.linalg.inv(A)
print(A)
print(A_inv)
print(A @ A_inv)
print(A_inv @ A)

v = np.array([1, 1])
print(v)
print(A @ v)

b1 = np.array([1,2,3])
b2 = np.array([3,4,5])
b3 = np.array([7,8,9])
B = np.array([b1, b2, b3]).transpose()
print(B)
print(np.ones(b1.shape))
print(np.tri(4))