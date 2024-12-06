import sympy as sp
import math

x, y, theta, xl, yl, rl, hl, fx, fy, cx, cy, tx, ty, tz = sp.symbols('x y theta xl yl rl hl fx fy cx cy tx ty tz')


T_om = sp.Matrix([[sp.sin(theta), -sp.cos(theta), 0, y*sp.cos(theta)-x*sp.sin(theta)],
                        [0, 0, -1, tz],
                        [sp.cos(theta), sp.sin(theta), 0, -x*sp.cos(theta)-y*sp.sin(theta)-tx],
                        [0, 0, 0, 1]])

xc = T_om[0,3]
yc = T_om[1,3]

psi = sp.atan2(yl-yc, xl-xc)

p1m = sp.Matrix([[xl-rl*sp.sin(psi)], [yl+rl*sp.cos(psi)], [0], [1]])
p2m = sp.Matrix([[xl-rl*sp.sin(psi)], [yl+rl*sp.cos(psi)], [hl], [1]])
p3m = sp.Matrix([[xl+rl*sp.sin(psi)], [yl-rl*sp.cos(psi)], [0], [1]])
p4m = sp.Matrix([[xl+rl*sp.sin(psi)], [yl-rl*sp.cos(psi)], [hl], [1]])

p1o = sp.simplify(T_om*p1m)
p2o = sp.simplify(T_om*p2m)
p3o = sp.simplify(T_om*p3m)
p4o = sp.simplify(T_om*p4m)

p1o = p1o.row_del(3)
p2o = p2o.row_del(3)
p3o = p3o.row_del(3)
p4o = p4o.row_del(3)

camera_proj_mat = sp.Matrix([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])

aux1 = sp.simplify(camera_proj_mat*p1o)
aux2 = sp.simplify(camera_proj_mat*p2o)
aux3 = sp.simplify(camera_proj_mat*p3o)
aux4 = sp.simplify(camera_proj_mat*p4o)

final_measurement= sp.simplify(sp.Matrix([[aux1[0]/aux1[2]], [aux1[1]/aux1[2]], [aux2[0]/aux2[2]],
                                                   [aux2[1]/aux2[2]], [aux3[0]/aux3[2]], [aux3[1]/aux3[2]],
                                                   [aux4[0]/aux4[2]], [aux4[1]/aux4[2]]]))

predict_measurement = sp.lambdify((x, y, theta, xl, yl, rl, hl, fx, fy, cx, cy, tx, ty, tz), final_measurement) #8 coordinates

Hx = final_measurement.jacobian(sp.Matrix([x, y, theta]))
#print(Hx.shape)

measurement_jac = sp.lambdify((x, y, theta, xl, yl, rl, hl, fx, fy, cx, cy, tx, ty, tz), Hx)
#x, y, theta, xl, yl, rl, hl, fx, fy, cx, cy, tx, ty, tz = 1,2,3,4,5,6,7,8,9,10,11,12,13,14
#predicted_pixels = predict_measurement(
#    x, y, theta, xl, yl, rl, hl, fx, fy, cx, cy, tx, ty, tz
#)
#print(predicted_pixels)
