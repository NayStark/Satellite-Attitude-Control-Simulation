% © Sunay Neelimathara
clear;
clc;
close all;

function qout = quatmultiply(q,r)
    w0 = q(1);
    x0 = q(2);
    y0 = q(3);
    z0 = q(4);
    w1 = r(1);
    x1 = r(2);
    y1 = r(3);
    z1 = r(4);
    qout = [w0*w1 - x0*x1 - y0*y1 - z0*z1;
            w0*x1 + x0*w1 + y0*z1 - z0*y1;
            w0*y1 - x0*z1 + y0*w1 + z0*x1;
            w0*z1 + x0*y1 - y0*x1 + z0*w1];
end

function qout = quatconj(q)
    qout = [q(1); -q(2:4)];
end

% Initial Conditions
I = diag([10, 15, 20]);
invI = inv(I);
q = [1; 0; 0; 0];
q_desired = [cos(pi/8); 0; sin(pi/8); 0];
omega = [0.5; -0.3; 0.2];

% PD Control gains
Kp = 20;
Kd = 10;

% Simulation settings
dt = 0.01;
T = 20;
N = T/dt;
time = zeros(1,N);
q_f = zeros(4,N);
omega_f = zeros(3,N);

q_f(:,1) = q; omega_f(:,1) = omega;

% Loop
for k = 1:N-1
    % Compute error quaternion (q_err = q_desired^-1*q)
    q_err = quatmultiply(quatconj(q_desired), q);

    % <PD control law>
    % --------------
    % tau = -Kp*q - Kd*w
    % --------------
    tau = -Kp*q_err(2:4) - Kd*omega;

    % <Dynamics>
    % I*dw/dt + crossproduct(w, (I*w)) = tau
    % Rearranged:
    % dw = inv(I)*(tau - crossproduct(w, I*w))
    domega = invI*(tau - cross(omega, I*omega));
    omega = omega + domega*dt;

    % <Quaternion kinematics>
    % capOmega should match:
    % [0   -wx  -wy  -wz
    %  wx    0   wz  -wy
    %  wy  -wz    0   wx
    %  wz   wy  -wx    0]
    capOmega = [0 -omega(1) -omega(2) -omega(3);
             omega(1) 0 omega(3) -omega(2);
             omega(2) -omega(3) 0 omega(1);
             omega(3) omega(2) -omega(1) 0];
    q = q + 0.5*capOmega*q*dt;
    q = q/norm(q);
    q_f(:,k+1) = q;
    omega_f(:,k+1) = omega;
    time(k+1) = (k+1)*dt;
end

% Plot results
figure(1);
subplot(2,1,1);
plot(time, q_f, 'LineWidth', 2);
xlabel('Time [s]'); ylabel('Quaternion');
legend('q_0','q_1','q_2','q_3');
title('Quaternion Evolution'); grid on;

subplot(2,1,2);
plot(time, omega_f, 'LineWidth', 2);
xlabel('Time [s]'); ylabel('\omega [rad/s]');
legend('\omega_x','\omega_y','\omega_z');
title('Angular Velocity'); grid on;
