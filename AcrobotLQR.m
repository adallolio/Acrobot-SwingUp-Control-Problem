% System definition
A = [0 0 1 0;
     0 0 0 1;
     49.47 -5.5 0 0;
     -50.01 66.23 0 0];
B = [0 0 -0.61 1.43]';
C = eye(4);
D = zeros(4,1);

% Q and R definition (Parameters for the controller)
Q = eye(4);
R = 1;

K = lqr(A,B,Q,R);
 
Ac = [(A-B*K)];
Bc = [B];
Cc = [C];
Dc = [D];

states = {'q1', 'q2', 'q1_dot','q2_dot'};
inputs = {'Torque'};
outputs = {'q1','q2','q1_dot','q2_dot'};

sys_cl = ss(Ac,Bc,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);

delta_t = 0.01;
t = 0:delta_t:1.8-delta_t;
T = 700;
Tc =T*ones(size(t));
[y,t,x]=lsim(sys_cl,Tc,t);

figure(1)
plot(t,y(:,1),'b',t,y(:,2),'r',t,y(:,3),'g',t,y(:,4),'k')
grid on
legend('q1','q2','q1d','q2d')


q1 = zeros(length(t),1);
q2 = zeros(length(t),1);
q1d = zeros(length(t),1);
q2d = zeros(length(t),1);
for i=2:1:length(t)
    q1dd = Ac(3,1)*(q1(i-1))+Ac(3,2)*q2(i-1)+Ac(3,3)*q1d(i-1)+Ac(3,4)*q2d(i-1)+Bc(3)*T;
    q2dd = Ac(4,1)*(q1(i-1))+Ac(4,2)*q2(i-1)+Ac(4,3)*q1d(i-1)+Ac(4,4)*q2d(i-1)+Bc(4)*T;
    q1d(i) = q1d(i-1)+delta_t*q1dd;
    q2d(i) = q2d(i-1)+delta_t*q2dd;
    q1(i) = q1(i-1)+delta_t*q1d(i);
    q2(i) = q2(i-1)+delta_t*q2d(i);
end

figure(3)
plot(t,q1,'b',t,q2,'r',t,q1d,'g',t,q2d,'k')
grid on
legend('q1','q2','q1d','q2d')
