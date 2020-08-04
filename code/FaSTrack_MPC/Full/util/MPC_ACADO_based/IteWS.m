function [state, control, runtime, out] = IteWS(x0,xF,N,Ts,TEB,stateWS,controlWS,A,b,dmin)
% Iteratively solve the MPC problem gradually increasing obstacle sizes

    BEGIN_ACADO;  

        %% parameters        
        % regularization parameter to improve numerical stability
        reg = 1e-4;
        
        %% set ACADO
        acadoSet('problemname', 'IteWS');               % Set your problemname. If you 
                                                        % skip this, all files will
                                                        % be named "myAcadoProblem"  
        
        %% defining optimization variables                    
        DifferentialState x;                            % DifferentialState
        DifferentialState vx;
        DifferentialState y;            
        DifferentialState vy;
        DifferentialState timeScale;
        DifferentialState L;

        Control ax;                                  % Control
        Control ay;    
        Control la1;                                 % dual multiplier associated with obstacleShape
        Control la2;
        Control la3;
        Control la4;
        Control la5;
        Control la6;
        Control la7;
        Control la8;
        Control la9;
        Control la10;
        Control la11;
        Control la12;
        
        f = acado.DiscretizedDifferentialEquation();
        
        %% cost function
        % (min time)+
        % (min states/control inputs)+      
        % (multiplier penalty)
        Q = blkdiag(0.1, 0.01, 0.1, 0.01);
        R = blkdiag(0.1, 0.1);
        f.add( next(L) == ...
            0.005*timeScale + 0.01*timeScale^2 +...
            [x-xF(1); vx; y-xF(2); vy]'*Q*[x-xF(1); vx; y-xF(2); vy] + [ax; ay]'*R*[ax; ay]+...
            reg*la1^2+reg*la2^2+reg*la3^2+reg*la4^2+reg*la5^2+reg*la6^2+reg*la7^2 +...
            reg*la8^2+reg*la9^2+reg*la10^2+reg*la11^2+reg*la12^2)
        
        %% dynamics of the car
        % - Double Integrator with forward Euler integration 
        % States:[x,y,vx,vy]  Control inputs:[ax ay]
        % - sampling time scaling, is identical over the horizon
        f.add(next(x)  == x + timeScale*Ts*vx);
        f.add(next(vx) == vx + timeScale*Ts*ax);
        f.add(next(y)  == y + timeScale*Ts*vy);        
        f.add(next(vy) == vy + timeScale*Ts*ay);
        f.add(next(timeScale) == timeScale);
        
        %% optimal control problem
        ocp = acado.OCP(0.0, N, N);                     % Set up the Optimal Control Problem (OCP) (start,duration,end)
        ocp.minimizeMayerTerm( L );                     % Minimize this Mayer Term
        ocp.subjectTo( f );                             % Your OCP is always subject to your 
                                                        % differential equation
        
        % bounds on states and inputs
        for i = 1:N
%             ocp.subjectTo( i, vx^2 + vy^2 <= (2.5-min(TEB))^2  );
            ocp.subjectTo( i, norm([vx;vy]) <= 2.5-max(TEB)  );
        end
        ocp.subjectTo( norm([ax;ay]) <= 1.0  );
        ocp.subjectTo( 0.01 <= timeScale <= 1.5  );
        ocp.subjectTo( la1 >= 0.0); ocp.subjectTo( la2 >= 0.0); ocp.subjectTo( la3 >= 0.0);
        ocp.subjectTo( la4 >= 0.0); ocp.subjectTo( la5 >= 0.0); ocp.subjectTo( la6 >= 0.0);
        ocp.subjectTo( la7 >= 0.0); ocp.subjectTo( la8 >= 0.0); ocp.subjectTo( la9 >= 0.0);
        ocp.subjectTo( la10 >= 0.0); ocp.subjectTo( la11 >= 0.0); ocp.subjectTo( la12 >= 0.0);
        
        %% initial and target states
        ocp.subjectTo( 'AT_START', L  == 0.0    );
        ocp.subjectTo( 'AT_START', x  == x0(1)  );
        ocp.subjectTo( 'AT_START', vx == x0(2)  );
        ocp.subjectTo( 'AT_START', y  == x0(3)  );        
        ocp.subjectTo( 'AT_START', vy == x0(4)  );        
%         ocp.subjectTo( 'AT_END', xF(1)-0.4 <=  x  <= xF(1)+0.4  );
%         ocp.subjectTo( 'AT_END', xF(2)-0.4 <=  y  <= xF(2)+0.4  );
        
        %% obstacle avoidance constraints
        A1 = A(1:4,:);	% extract obstacle matrix associated with j-th obstacle
        b1 = b(1:4);	% extract obstacle matrix associated with j-th obstacle
        A2 = A(5:8,:);
        b2 = b(5:8);
        A3 = A(9:12,:);
        b3 = b(9:12);

        for i = 2:N     % Ignore the constraints at the first and second shooting index, for feasibility        
            %% norm(A'*lambda) <= 1
            ocp.subjectTo( i, (A1'*[la1;la2;la3;la4])'*(A1'*[la1;la2;la3;la4]) <= 1 );
            ocp.subjectTo( i, (A2'*[la5;la6;la7;la8])'*(A2'*[la5;la6;la7;la8]) <= 1 );
            ocp.subjectTo( i, (A3'*[la9;la10;la11;la12])'*(A3'*[la9;la10;la11;la12]) <= 1 );

            %% -g'*mu + (A*p - b)*lambda > 0
            ocp.subjectTo( i, (A1*[x;y]-b1)'*[la1;la2;la3;la4] >= dmin );
            ocp.subjectTo( i, (A2*[x;y]-b2)'*[la5;la6;la7;la8] >= dmin );
            ocp.subjectTo( i, (A3*[x;y]-b3)'*[la9;la10;la11;la12] >= dmin );
        end
        
        %% set up the optimization algorithm
        algo = acado.OptimizationAlgorithm( ocp );
        algo.set( 'MAX_NUM_ITERATIONS', 50 );
        algo.set( 'KKT_TOLERANCE', 1e-1 );
        algo.set( 'INTEGRATOR_TYPE', 'INT_RK45');
    
        %% set initial guess
        STATES_INIT = zeros(N+1,7);
        STATES_INIT(:,1:7) = stateWS(:,1:7);
        if length(controlWS(1,:)) ~= 15  % 15 or 27
            CONTROLS_INIT = zeros(N+1,15);
            CONTROLS_INIT(:,1:3) = controlWS;
        else
            CONTROLS_INIT = controlWS;
        end
        algo.initializeDifferentialStates(STATES_INIT);
        algo.initializeControls(CONTROLS_INIT);
                        
    END_ACADO;                                          % Always end with "END_ACADO".
                                                        % This will generate a file problemname_ACADO.m. 
                                                        % Run this file to get your results. 
                                                        % You can run the file problemname_ACADO.m as
                                                        % many times as you want without having to compile again.
    tic()
    out = IteWS_RUN();                                  % Run the test. The name of the RUN file
                                                        % is problemname_RUN, so in
                                                        % this case active_damping_RUN
    runtime = toc();

	%% return values
    fprintf('Elapsed time: %f\n',runtime);
    state = out.STATES;
    control = out.CONTROLS;
end