import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply


q1 = np.array([0.0, 0.0, 0.0, 1.0])
q2 = quaternion_from_euler(0.0, 0.0, np.pi/2.0)
q3 = quaternion_multiply(q1, q2)
print(euler_from_quaternion(q3))