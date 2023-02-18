from tf.transformations import euler_from_quaternion


x = -2.433142812681704e-05
y = 0.0015890096367740956
z = 0.015042774875815802
w = 0.9998855881451594

roll, pitch, theta = euler_from_quaternion([x, y, z, w])
print(theta)
