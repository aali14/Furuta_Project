## Furuta Pendulum

Project Team Members: 
Ali H. Ali - Keenan Jacob - ChulWoong Kang - Ashby Saldivar - Xuan Yuan

### Introduction

The problem given is to create a testing platform for the control system controls of a furuta pendulum or "rotation inverted prendulum". Our team must create a mathemtaical model of the given system along side a composed control system. To simplify this process further one must create a model in CoppelliaSim from the mathemtical model, then create code from matlab to control the model in CoppeliaSim, and lastly collect data from the simulation in CoppeliaSim within MATLAB.

![image](https://user-images.githubusercontent.com/65076893/82099527-6a520680-96bc-11ea-9868-1ad7c26ac722.png)


### Modeling 


### Appendix A: Simulation Code
```Introduction
clc
clear
clf
disp('Program started');
sim=remApi('remoteApi'); 
sim.simxFinish(-1); 
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);
g = 9.81;
mp = 3.902;
mr = 3.902;
Jp = .02145;
Jr = .02145;
Lp = .55;
Lr = .65;

if (clientID>-1)
    A = [0 1 0 0;
        0 0 -mp^2*Lp^2/4*Lr*g/(Jr*(Jp+mp*Lp^2/4)+Jp*mp*Lr^2) 0;
        0 0 0 1;
        0 0 (Jr+mp*Lr^2)*mp*Lp/2*g/((Jp+mp*Lp^2/4)+Jp*mp*Lr^2) 0];
    B = [0;
        (Jp+ mp*Lp^2/4)/(Jr*(Jp+mp*Lp^2/4)+Jp*mp*Lr^2);
        0;
        -mp*Lp/2*Lr/(Jr*(Jp+mp*Lp^2/4)+Jp*mp*Lr^2)];
    eigs = [-8.2; -8.3; -8.4; -8.5];
    K = place(A,B, eigs);

    
  disp('Connected to remote API server');
    [returnCode]=sim.simxStartSimulation(clientID,sim.simx_opmode_oneshot);
    [returnCode,penbearing]=sim.simxGetObjectHandle(clientID,'RevoluteJoint2',sim.simx_opmode_blocking);
    [returnCode,pen]=sim.simxGetObjectHandle(clientID,'Cylinder_3',sim.simx_opmode_blocking);
    [returnCode,act1]=sim.simxGetObjectHandle(clientID,'Actuator_1',sim.simx_opmode_blocking);
    [returnCode,act2]=sim.simxGetObjectHandle(clientID,'Actuator_2',sim.simx_opmode_blocking);
    [returnCode,arm]=sim.simxGetObjectHandle(clientID,'Cylinder_2',sim.simx_opmode_blocking);
    [returnCode,armbearing]=sim.simxGetObjectHandle(clientID,'RevoluteJoint1',sim.simx_opmode_blocking);
    
    %Pulls the two linear actuators up at 2 m/s
    [returnCode]=sim.simxSetJointTargetVelocity(clientID,act2,2,sim.simx_opmode_streaming);
    [returnCode]=sim.simxSetJointTargetVelocity(clientID,act1,2,sim.simx_opmode_streaming);
    
    
    t=clock;
    startTime=t(6);
    currentTime=t(6);

    [returnCode,armangle]=sim.simxGetJointPosition(clientID,arm,sim.simx_opmode_streaming);
    [returnCode,linearvelarm,angularvelarm]=sim.simxGetObjectVelocity(clientID,arm,sim.simx_opmode_streaming);
    [returnCode,penangle]=sim.simxGetJointPosition(clientID,penbearing,sim.simx_opmode_streaming);
    [returnCode,linearvelpen,angularvelpen]=sim.simxGetObjectVelocity(clientID,pen,sim.simx_opmode_streaming);
    

    i = 1;
    while (currentTime-startTime < 8) 
        [returnCode,armangle]=sim.simxGetJointPosition(clientID,armbearing,sim.simx_opmode_streaming);
        [returnCode,linearvelarm,angularvelarm]=sim.simxGetObjectVelocity(clientID,arm,sim.simx_opmode_streaming);
        [returnCode,penangle]=sim.simxGetJointPosition(clientID,penbearing,sim.simx_opmode_streaming);
        [returnCode,linearvelpen,angularvelpen]=sim.simxGetObjectVelocity(clientID,pen,sim.simx_opmode_streaming);
        if (returnCode==sim.simx_return_ok) % After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
            
                timeMatrix(i,1) = currentTime;
                timeMatrix(i,2) = armangle;
                timeMatrix(i,3) = angularvelarm(3);
                timeMatrix(i,4) = -penangle;
                timeMatrix(i,5) = atan(tan(-angularvelpen(1))/cos(armangle));
                thetadot = -atan(tan(-angularvelpen(1))/cos(armangle));
                
                if penangle>.01 && penangle<.99*pi()
                    torque = 2*K*[pi()-armangle;angularvelarm(3);penangle;thetadot];
                    [returnCode]=sim.simxSetJointTargetVelocity(clientID,armbearing,-5,sim.simx_opmode_streaming);
                elseif penangle>-.99*pi() && penangle<-.049
                    torque = 2*K*[pi()-armangle;angularvelarm(3);penangle;thetadot];
                    [returnCode]=sim.simxSetJointTargetVelocity(clientID,armbearing,5,sim.simx_opmode_streaming);
                else
                    torque = 0;
                end
                
                timeMatrix(i,6) = torque;
                [returnCode]=sim.simxSetJointForce(clientID,armbearing,torque,sim.simx_opmode_oneshot);
                i = i+1;
                
        end
        
        t=clock;
        currentTime=t(6);
    end

    % Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    sim.simxGetPingTime(clientID);
    sim.simxFinish(clientID); 
        
else
        disp('Failed connecting to remote API server');
end

sim.delete(); % call the destructor!
disp('Program ended');
plot(timeMatrix(:,1), timeMatrix(:,4))
hold on;
%plot(timeMatrix(:,1), timeMatrix(:,3))
%plot(timeMatrix(:,1), timeMatrix(:,4))
plot(timeMatrix(:,1), timeMatrix(:,6))
%legend('penangle', 'penvelocity(m/s)', 'armvel', 'armposition')
```

### Controller Design and Simulations


For this project, the control of the dynamics was to be implemented through CoppeliaSim (formerly V-Rep). Coppelia allows a user to create a system using simple blocks while accurately computing dynamic properties such as moment of intertia based on some user imput.

In order to control the pendulum, we connected Matlab with Coppelia using Coppelia's remote Api capability for Matlab. This was beneficial for our group as we are more familiar with Matlab syntax. Unfortunately, Matlab cannot send input to Coppelia until the simulation starts. This means that even if the pendulum initial position is up, it will fall over before the control from Matlab is sent. To prevent this, a gate type system placed to stop the pendulum from falling, then moved away once the control from Matlab was implemented.

![image](https://user-images.githubusercontent.com/35712553/82102407-514d5380-96c4-11ea-901d-b92578075746.png)
Figure 2: 

For this project, we had control over the torque of the center motor. In order to determine the torque that needs to be applied, the K matrix must be solved for. 

dx/dt = (A - BK)x
and
torque = Kx


In order to solve for the K matrix, we used Matlab's place function. This function is only valid if the rank of the controllablity matrix is equal to the number of state variables. A quick function used to varify this condition in Matlab was:

`Controlrank = rank(ctrb(A,B))`

Since this value was equal to 4, we found the gain (K) by the following:

`K = eig(A,B,eigs)`

were eigs is matrix of the desired poles of the system.

With K known, the next step is to collect instantaneous values of the angle and angular velocity of the revolute joints. To do this, we used Coppelia's remoteApi get commands, which collected the data of  θ, φ, dθ/dt, and dφ/dt. Without any torque, the values below were recorded.

![image](https://user-images.githubusercontent.com/35712553/82104771-04ba4600-96cd-11ea-836b-13811ae16e48.png)
Figure 3: Plot of values retrieved from Coppelia with no torque applied

With the state space variables being collected almost instantaneously, we can multiply our gain, K, to solve for the required torque at each point. 

### Results

Using the Coppelia model, we were not able to balance the pendulum in the up position. It seemed that the torque was not applied fast enough to combat the pendulum falling. There are a multitude of reasons this could be the case. It is possible that since Matlab and Coppelia were not linked in Synchronous mode, the connection between the two was not fast enough. Another possible reason for the lack of balance could be that some of the model properties were not correct. Although we attempted to combat these problems by adjusting input values (i.e. the torque/angle), we were unsuccessful. 





