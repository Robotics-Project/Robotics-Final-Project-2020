L1 = 25;
L2 = 25;
px = 5.92;
py = -4.55;

th2 = acos((py^2+px^2-L1^2-L2^2)/(2*L1*L2))
rad_th2 = rad2deg(th2)

th1 = (-py*L2*sin(th2)+px*(L1+L2*cos(th2)))/(px*L2*sin(th2)+py*(L1+L2*cos(th2)))
rad_th1 = rad2deg(th1)