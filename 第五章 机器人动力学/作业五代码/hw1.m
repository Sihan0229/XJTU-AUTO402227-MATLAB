open ode45
mdl_puma560
q = [-60, 90, 30,0,0,0]*pi/180;
qd = [0 0 0 0 0 0];
t = [1 1 1 1 1 1];
qdd = p560.accel(q,qd,t)
figure
view(3);
p560.plot(q);