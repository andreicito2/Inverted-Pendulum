pkg load control sockets

%System Parameters
m = 0.0318;   % Pendulum Mass (kg)
M = 0.3333;   % Cart Mass (kg)
l = 0.316/2;  % Half Length of Pendulum (m)
g = 9.81;     % Gravity (m/s^2)
I = 0.0085*(0.0098^2+0.0379^2)/12 + m*((l*2)^2)/3; % Rotational Inertia of Pendulum (kg*m^2)

a1 = 0.0185;
c1 = 2*a1*I;      % Viscous friction of pendulum 1 (rotational) (Nms/rad)

alpha = 12.2;      % Carriage slope (deg)
xdotss = 0.4852;   % Terminal velocity (m/s)
c = (M+m)*g*sin(alpha*pi/180)/xdotss;   % Damping / Viscous Friction (kg/s)


%% Setup Motion Matrices (xdot = Ax + Bu)
denom = (M+m)*(m*l^2+I);

A = [0 1 0 0;
    0 -(m*l^2+I)*c/denom m^2*l^2*g/denom m*l*c1/denom;
    0 0 0 1;
    0 -m*l*c/denom (M+m)*m*l*g/denom (M+m)*c1/denom];

B = [0;
    (m*l^2+I)/denom;
    0
    m*l/denom];

%% Design LQR controller
Q = diag([50000 0 100 0]); % Weight of each variable (x, xdot, Theta1, Theta1dot)
R = 1;                     % Motor control cost
K = lqr(A,B,Q,R);          % State feedback matrix


%% Create observer
C = [1 0 0 0;
    0 0 1 0];

eigs = [-40 -41 -42 -43];  % Observer Poles
L = place(A',C',eigs)';    % Observer Gain Matrix

Ad = A-L*C


%% Control Motor
ctrlbox;              % load ctrlbox comm functions

% Define time and sample settings
T=1/1000;             % Period (s)
Trun = 10;            % Run time (s)
cnt=Trun/T;           % Number of times through loop
srate = 1/T;          % Sample rate (Hz)

% Define encoder scaling
rd = 0.0254/2;        % Drive pulley radius (m)
encpts = 4096;        % Number of encoder measurement points
scale = [-rd*2*pi/encpts  -2*pi/encpts];  % Define encoder scaling

% Initialize Matrices
store = zeros(cnt,5);   % Storage matrix
rdata = [0,0,0,0];      % Receive data [Theta1, Theta2, x, unused]
xhat = [0; 0; 0; 0];    % Control data [x, xdot, Theta1, Theta1dot]


disp(' Rotate the long pendulum CCW to vertical and hit ENTER.')
pause;

ctrlbox_init();            % Connect to FPGA control board
disp('finished init');     % Confirm connection

period = 1000000./srate;   % Define sample period
ctrlbox_send(0,0,period);  % Send sample period
disp('finished send');     % Confirm send

while (1)
    tic;
      for c=1:cnt
        % read encoder values
        rdata = ctrlbox_recv();  % Receive data [Theta1, Theta2, x, unused]

        % if something failed, display error and loop count
        if (ctrlbox_error() != 0)
            fprintf('ctrlbox_error: %d\n',ctrlbox_error());
            ctrlbox_shutdown();
            return;
        end

        xhat(:,c+1)=T*(Ad*xhat(:,c)-B*K*xhat(:,c)+L*([rd*rdata(3).*scale(1);rdata(1)*scale(2)-pi]))+xhat(:,c);

        % Generate pwm value
        pwm = ((-K*xhat(:,c+1))*32768)/20;

        % Write pwm values and enable motor
        ctrlbox_send(pwm, 1, 0);

        % Force matlab to check for interrupts and flush event queue
        drawnow;

        % Save data
        store(c,:) = [rdata(1).*scale(2),rdata(3).*scale(1),xhat(:,c+1)(2),xhat(:,c+1)(4),0];
	      % store(c,:) = [rdata,pwm];
      end
    runtime = toc;
    fprintf('transactions=%d seconds=%d transactions/sec=%f\n',
        c, runtime, c/runtime);
    drawnow;
end

% Disable motor and disconnect
ctrlbox_shutdown();
