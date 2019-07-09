figure(1)
hold all
t1 = myPID601030nocutoff(:,1);
x1 = myPID601030nocutoff(:,2);
plot(t1,x1,'LineWidth',1.5)
figure(1)
hold all
plot(t1,x1,'LineWidth',1.5,'AlignVertexCenters','on')
figure(1)
hold all
plot(t1,x1,'LineWidth',1.5)
grid on
t2 = ROSpid601030(:,1);
x2 = ROSpid601030(:,2);
plot(t2,x2,'LineWidth',1.5)
figure(2)
hold all
grid on
t3 = myPID60153010(:,1);
x3 = myPID60153010(:,2);
plot(t3,x3,'LineWidth',1.5)
plot(t3,x3,'LineWidth',1.5)
t4 = ROSpid601530(:,1);
x4 = ROSpid601530(:,2);
plot(t4,x4,'LineWidth',1.5)
legend('myPID 60 10 30 no cutoff','ROSpid 60 10 30','my PID 60 15 30 10','ROSpid 60 15 30')
t5 = myPID601030andKa10(:,1);
x5 = myPID601030andKa10(:,2);
plot(t4,x4,'LineWidth',1.5)
legend('myPID 60 10 30 no cutoff','ROSpid 60 10 30','my PID 60 15 30 10','ROSpid 60 15 30','myPID 60 10 30 Ka10')
