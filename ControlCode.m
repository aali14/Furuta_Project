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
        0 0 -mp^2*(Lp^2)/4*Lr*g/(Jr*(Jp+mp*(Lp^2)/4)+Jp*mp*Lr^2) 0;
        0 0 0 1;
        0 0 (Jr+mp*(Lr^2))*mp*Lp/2*g/((Jp+mp*(Lp^2)/4)+Jp*mp*Lr^2) 0];
    B = [0;
        (Jp + mp*Lp^2/4)/(Jr*(Jp+mp*(Lp^2)/4)+Jp*mp*Lr^2);
        0;
        -mp*Lp/2*Lr/(Jr*(Jp+mp*Lp^2/4)+Jp*mp*Lr^2)];
    eigs = [-1.4; -1.5; -1.7; -1.8];
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
plot(timeMatrix(:,1), timeMatrix(:,2))
hold on;
plot(timeMatrix(:,1), timeMatrix(:,6))
%plot(timeMatrix(:,1), timeMatrix(:,4))
%plot(timeMatrix(:,1), timeMatrix(:,5))
%legend('arm angle, (rad)', 'arm velocity, (rad/s)','pendulum angle, (rad)', 'pendulum velocity, (rad/s)')
legend('arm angle, rad)', 'applied torque')
xlabel('time (s)')
