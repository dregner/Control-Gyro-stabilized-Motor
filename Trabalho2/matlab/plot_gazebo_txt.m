figure
subplot(3,1,1)
plot(kalman(:,1),kalman(:,5))
hold on
plot(kalman(:,1),kalman(:,2))
legend('x1','x1e')
xlabel('tempo s')
subplot(3,1,2)
plot(kalman(:,1),kalman(:,6))
hold on
plot(kalman(:,1),kalman(:,3))
legend('x2','x2e')
xlabel('tempo s')
subplot(3,1,3)
plot(kalman(:,1),kalman(:,7))
hold on
plot(kalman(:,1),kalman(:,4))
legend('x3','x3e')
xlabel('tempo s')