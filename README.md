## Furuta Pendulum

Project Team Members: 
Ali H. Ali - Keenan Jacob - ChulWoong Kang - Ashby Saldivar - Xuan Yuan

### Introduction

The problem given is to create a testing platform for the control system controls of a furuta pendulum or "rotation inverted prendulum". Our team must create a mathemtaical model of the given system along side a composed control system. To simplify this process further one must create a model in CoppelliaSim from the mathemtical model, then create code from matlab to control the model in CoppeliaSim, and lastly collect data from the simulation in CoppeliaSim within MATLAB.

Couple of the resources used to create the project were I. Fantoni and R. Lozano, Nonlinear Control of Underactuated Mechanical Systems, Springer, London, UK, 2002 particularly chapter 6 that covers find the eqautions of motion by using the Euler-Lagrange formulation. Another resource was the Professor Bank’s CoppeliaSim resource videos used to create the model in CoppeliaSim and properly arranging the model to run well during simulations. The last resource used was the manual on API functions with CoppeliaSim by Coppelia Robotics to create code with CoppeliaSim for the model and to connect with MATLAB.

           ![image](https://user-images.githubusercontent.com/65076893/82099527-6a520680-96bc-11ea-9868-1ad7c26ac722.png)


         [![Watch the video](https://user-images.githubusercontent.com/65076893/82101816-6c1ec880-96c2-11ea-87fd-12835277af05.jpg)]                                                  (https://www.youtube.com/watch?v=XKzzWe15DEw)
Sample Video


### Modeling 
The calculations of the Furuta Pendulum were based on the Lagrange method by using the total energies of the system.


### Controller Design and Simulations

For this project, the control of the dynamics was to be implemented through CoppeliaSim (formerly V-Rep). Coppelia allows a user to create a system using simple blocks while accurately computing dynamic properties such as moment of intertia based on some user imput. 
Since the state variable to control consisted of θ, φ, dθ/dt, and dφ/dt, those variable were collected 


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






