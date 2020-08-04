function [state, control, runtime] = p2pWS(x0,xF,N,Ts)
% Computes trajectory ignoring the obstacles, used for warm-start

    BEGIN_ACADO;  

        acadoSet('problemname', 'p2pWS');               % Set your problemname.
        
        %% defining optimization variables                          
        DifferentialState x;                            % DifferentialState
        DifferentialState vx;
        DifferentialState y;            
        DifferentialState vy;
        DifferentialState timeScale;
        DifferentialState L;
        Control ax;                                     % Control
        Control ay;
        
        f = acado.DiscretizedDifferentialEquation(Ts);

        %% cost function
        % (min time)+
        % (min states/control inputs)      
        Q = blkdiag(0,0,0,0);
        R = blkdiag(0.1,0.1);
        f.add( next(L) == 0.5*timeScale + 1*timeScale^2 +...
            [x; vx; y; vy]'*Q*[x; vx; y; vy] + [ax; ay]'*R*[ax; ay]);

        %% dynamics
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

        %% bounds on states and inputs
%         ocp.subjectTo(  vx^2 + vy^2 <= 2.0  );
        ocp.subjectTo(  ax^2 + ay^2 <= 0.81  );
%         ocp.subjectTo( -0.6 <= ax   );
%         ocp.subjectTo( -0.6 <= ay   );
%         ocp.subjectTo( -0.2 <= ax <= 1.0  );
%         ocp.subjectTo( -0.2 <= ay <= 1.0  );
        ocp.subjectTo( 0.01 <= timeScale <= 5.0  );

        %% start and finish point
        ocp.subjectTo( 'AT_START', L  == 0.0     );
        ocp.subjectTo( 'AT_START', x  == x0(1)   );
        ocp.subjectTo( 'AT_START', vx == x0(2)   );
        ocp.subjectTo( 'AT_START', y  == x0(3)   );        
        ocp.subjectTo( 'AT_START', vy == x0(4)   );
        ocp.subjectTo( 'AT_END',   x  == xF(1)   );
        ocp.subjectTo( 'AT_END',   y  == xF(2)   );

        %% set up the optimization algorithm
        algo = acado.OptimizationAlgorithm( ocp );
        algo.set( 'MAX_NUM_ITERATIONS', 750 );
        algo.set( 'KKT_TOLERANCE', 1e-1 );
        algo.set( 'INTEGRATOR_TYPE', 'INT_RK45');
    %     algo.set( 'INTEGRATOR_TOLERANCE',   1e-3);  
    %     algo.set( 'ABSOLUTE_TOLERANCE',   1e-3   );

        %% set initial guess
        xINIT = zeros(N+1,7);   % Order: timestamp,x,y,vx,vy,timeScale,L
        xINIT(:,1) = 0:1:N;
        xINIT(:,2) = linspace(x0(1),xF(1),N+1);
        xINIT(:,4) = linspace(x0(3),xF(2),N+1);
        xINIT(:,6) = 0.5*ones(N+1,1);
        
        uINIT = zeros(N,3);     % Order: timestamp,ax,ay
        uINIT(:,1) = 0:1:N-1;
%     	uINIT(:,2) = 0.6*ones(N,1);
%       uINIT(:,3) = 0.6*ones(N,1);
    	uINIT(:,2) = zeros(N,1);
        uINIT(:,3) = zeros(N,1);
        algo.initializeDifferentialStates(xINIT);
%         algo.initializeControls(uINIT);

    END_ACADO;                                      % Always end with "END_ACADO".
                                                    % This will generate a file problemname_ACADO.m. 
                                                    % Run this file to get your results. 
                                                    % You can run the file problemname_ACADO.m as
                                                    % many times as you want without having to compile again.
    tic()
    out = p2pWS_RUN();                              % Run the test. The name of the RUN file
                                                    % is problemname_RUN, so in
                                                    % this case active_damping_RUN
    time = toc();

    %% return values
    state = out.STATES;
    control = out.CONTROLS;
    runtime = time;
    
end