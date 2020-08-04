function [nextState,nextControl,EndFlag] = findNextStateODE(x0,mpc,dt,simTime, TEB_list, iter, iterMax)
% find next state [x,vx,y,vy] to track
    
    EndFlag = false;

    mpcState = mpc.stateOpt;
    mpcControl = mpc.controlOpt;
    
    i = find(mpcState(:,1)<=simTime,1,'last');
%     simTime
%     mpcState(i,1)
    if i == length(mpcState(:,1))
        EndFlag = true;
        pause
    end
    ax = mpcControl(i,2);
    ay = mpcControl(i,3);
    
    vxNext = x0(2)+dt*ax;    
    vyNext = x0(4)+dt*ay;
    
%     % Detect speed overshooting
%     spdLimitSQU = min(TEB_list)^2;
%     vNextSQU = vxNext^2+vyNext^2;
%     
%     if (vNextSQU >= spdLimitSQU) && (mod(iter,iterMax) == iterMax-1)    % Normalize vx and vy, for feasibility
%         vxNext = vxNext/sqrt(vNextSQU)*sqrt(spdLimitSQU);
%         vyNext = vyNext/sqrt(vNextSQU)*sqrt(spdLimitSQU);
%         if sqrt(spdLimitSQU)/sqrt(vNextSQU) < 0.98
%             fprintf('[WARN]: Speed limit exceeded, normalizing by %f \n',sqrt(spdLimitSQU)/sqrt(vNextSQU))
%         end
%     end
    
    % Evolve x and y
    xNext = x0(1)+dt*vxNext;
    yNext = x0(3)+dt*vyNext;
    
    % One step simulation
    nextState = [xNext; vxNext; yNext; vyNext];
    nextControl = [vxNext;vyNext];
    
end