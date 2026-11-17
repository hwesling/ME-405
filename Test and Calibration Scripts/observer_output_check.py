""" MECHA 15 - Harry """
""" Temp Lab 9 File to compare MCU observer output to MATLAB observer simulation """

from ulab import numpy as np

# Paste MATLAB arrays
Ao = np.array([
    [3.550270598904896e-05, -1.5352171978371274, 0.0048798267467947748],
    [0.0048798267467947704, 0.7483046398689529, 0.00022018644547564163],
    [0, -99.029067144029568, 1],
])
Bo = np.array([
    [2.3499165686409258, 1.5352171978371274],
    [0.10603240714503231, 0.2516953601310471],
    [0, 99.029067144029568],
])

Co = np.array([
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1],
])
Do = np.array([
    [0, 0],
    [0, 0],
    [0, 0],
])

# Values used in the MATLAB live script
x_check = np.array([1.0, 0.2, -0.5])
w_check = np.array([2.0, 0.21])

# Use np.dot() inistead of @ in ulab
y_check = np.dot(Co, x_check) + np.dot(Do, w_check)
x_next_check = np.dot(Ao, x_check) + np.dot(Bo, w_check)

print("y_check:")
print(y_check)

print("x_next_check:")
print(x_next_check)

