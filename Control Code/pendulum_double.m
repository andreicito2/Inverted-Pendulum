% Tyler Sing and Andrei Maiorov: ENGR 454 double inverted Pendulum Project
pkg load control sockets

%% System Parameters
m1 = 0.0318; %Long Pendulum(1) mass
m2 = 0.0085; %Short Pendulum(2) mass
M = 0.3163; %Cart Mass
l1 = 0.316/2; %Half length of Pendulum(1)
l2 = 0.079/2; %Half length of Pendulum(2)
g = 9.81; %Gravity
I1 = 0.0085*(0.0098^2+0.0379^2)/12 + m1*((l1*2)^2)/3;
I2 = 0.0085*(0.0098^2+0.0379^2)/12 + m2*((l2*2)^2)/3;


alpha = 12.2; %Carriage Slope
xdotss = 0.4852; %Terminal Velocity

a1 = 0.0185;
a2 = 0.012;

c = (M+m1+m2)*g*sin(alpha*pi/180)/xdotss; %Damping / Viscous Friction
c1 = 2*a1*I1; % Nms/rad    Viscous friction of pendulum 1 (rotational
c2 = 2*a2*I2; % Nms/rad    Viscous friction of pendulum 2 (rotational


% Mass component
M = [ M+m1+m2 m1*l1      m2*l2;
    -m1*l1     -m1*l1^2+I1 0 ;
    -m2*l2     0          -m2*l2^2+I2];

inv_M = inv(M)

A = [0 1 0 0 0 0;
    0 -inv_M(1,1)*c inv_M(1,2)*m1*l1*g inv_M(1,2)*c1 inv_M(1,3)*m2*l2*g inv_M(1,3)*c2;
    0 0 0 1 0 0;
    0 -inv_M(2,1)*c inv_M(2,2)*m1*l1*g inv_M(2,2)*c1 inv_M(2,3)*m2*l2*g inv_M(2,3)*c2;
    0 0 0 0 0 1;
    0 -inv_M(3,1)*c inv_M(3,2)*m1*l1*g inv_M(3,2)*c1 inv_M(3,3)*m2*l2*g inv_M(3,3)*c2;

B = [0; inv_M(1,1); 0; inv_M(2,1); 0; inv_M(3,1)];

%% Design LQR controller
Q = diag([1000, 0, 100, 0, 100, 0]); % Weights of each variable

R = 1;   % Cost of using motor

K = lqr(A,B,Q,R);

%% Observer
C = [1 0 0 0 0 0;
    0 0 1 0 0 0;
    0 0 0 0 1 0];

eigs = [-10,-11,-12,-13,-14,-15];

L = place(A',C',eigs)';

Ad = A-L*C;

D = zeros(size(C,1),size(B,2));

% Control Motor
ctrlbox;        % load ctrlbox comm functions

disp(' Rotate both pendulums CCW to vertical and hit ENTER.')

pause;


T=1/1000;       % Sample period (s)
Trun = 10;      % Run time (s)
cnt=Trun/T;     % Number of times through loop
srate = 1/T;    % Sample rate (Hz)

store = zeros(cnt,5);
rdata = [0,0,0,0];        % receive data [
rd = 0.0254/2; % m         Drive pulley radius
scale = [-rd*2*pi/4096  -2*pi/4096 -0.05/250];

xhat=[0; 0; 0; 0; 0; 0];


    ctrlbox_init();

    disp('finished init');

    % send sample period
    period = 1000000./srate;

    ctrlbox_send(0,0,period);

    disp('finished send');

while (1)
    x = 1:cnt;
    tic;
      for c=1:cnt
        % read encoder values
        rdata = ctrlbox_recv();

        % if something failed, display error and loop count
        if (ctrlbox_error() != 0)
            fprintf('ctrlbox_error: %d\n',ctrlbox_error());
            %disp('last error:');
            %disp(lasterror.message);
            ctrlbox_shutdown();
            %exit();
            return;

	% elseif c > 1 && rdata(3) < 3*pi/4 || rdata(3) > 5*pi/4
	%    fprintf('Error. Angle out of range');
	%    ctrlbox_shutdown();
	%    return;

	end

        xhat(:,c+1)=T*(Ad*xhat(:,c)-B*K*xhat(:,c)+L*([rd*rdata(3).*scale(1);rdata(1)*scale(2)-pi;rdata(2)*scale(2)-pi]))+xhat(:,c);

        % pwm generation
        pwm = ((-K*xhat(:,c+1))*32768)/20;

        % write pwm valuesm and enable motor
        ctrlbox_send(pwm, 1, 0);

        % force matlab to check for interrupts and flush event queue
        drawnow;

        % save data
        % store(c,:) = [rdata(1).*scale(2),rdata(3).*scale(1),xhat(:,c+1)(2),xhat(:,c+1)(4),0];
	      store(c,:) = [rdata,pwm];
      end
    runtime = toc;
    fprintf('transactions=%d seconds=%d transactions/sec=%f\n',
        c, runtime, c/runtime);
    drawnow;
end

% disable motor and disconnect
ctrlbox_shutdown();


