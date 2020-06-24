% While running a simulation, place a breakpoint within costFncCtrl and
% execute these lines to check the prediction quality of estimated model

close all

Y(:, 1) = y;
for k = 2:N
    Y(:, k) = Y(:, k-1) + dt * f(Y(:, k-1), myU(:,k-1)); % Euler scheme. May be improved to more advanced numerical integration
end

plot(0:dt:(N-1)*dt, Y(1:3,:)', 'b')
hold all

Y(:, 1) = y;
x = x0;
for k = 2:N
    x = A*x + B*myU(:, k-1);
    Y(:, k) = C*x + D*myU(:, k-1);
end

plot(0:dt:(N-1)*dt, Y(1:3,:)', '--r')