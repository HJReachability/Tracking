%% MpcPlanner
classdef MpcPlanner < handle
  % *Description:* MAIN FUNCTION: This function is a basic implementation of
  % the model predictive control (MPC) path planning algorithm with
  % optimization-based collision avoidance approach [Zhang et al. 2017]
  %
  % *Author:* Haimin Hu
  %
  % *Last Updated:* 4th August 2020
  %
  % *Features:*
  %   - 2D search space
  %   - Obstacle avoidance with polytopic formulation
  %   - Path smoothing
  
  properties
    %% RUN OPTIONS
    %  ************************************************************************
    % Display                           => Activate visual display, SLIGHTLY SLOWER
    doDraw=false;
    
    % Skip this number of drawings before doing an update to the figure (speeds up plotting)
    drawingSkipsPerDrawing = 10;
    
    %% VARIABLES
    %  ************************************************************************
    % The main data structure
    mpc;
    
    % Default Start and Goal  => Point [x y]
    x0=[0;0;0;0];
    xF =[10;0;10;0];        
    
    % Predoction horizon for iterative warmstart and final optimization
    N = 5;
    N_opt = 15;
    
    % safety distance
    dmin = 0.3;
    
    % Nominal sampling interval
    Ts = 1.0;
    
    % Plant is modelled as a square with length PlantL
    PlantL = 0.2;
    
    % MPC outputs of state and control sequences
    stateP2P = [];
    controlP2P = [];
    stateIteWS = {};
    controlIteWS = {};
    stateOpt = [];
    controlOpt= [];
    
    % Obstacle in lOb representation (check getlOb.m)
    lOb;
    
    % Number of single obstacles and their vertices
    nOb = [4 4 4];
    
    % Whether to perform a final optimization
    isFinalOpt = true;
    
    % Scaling range for iterative warmstart
    ScaleRange = 0.4:0.2:1.0;
    
    % Figure handle
    figure_h = [];
    
    % The number of drawing that have been skipped since the last draw command
    skippedDrawings = 0;    
    
    % The handles for the lines and points in the figure
    plotHandles = [];
    
    % The path from x0 to xF
    path = [];
    
    % Current time
    tt = 0;
    
    % Current TEB list
    TEB = [];
    
    % TEB sampling interval
    tau = 0.2;
    
    % Runtime
    runtimeP2P = 0;
    runtimeIteWS = 0;
    runtimeOpt = 0;
    
    % Handles for plots
    hT % Final MPC trajectory
    hE % Ending (target) postion
    hSO % Scaled obstacles
    hST % Trajecotry associated with scaled obstacles
    
  end
  
  properties (SetAccess=protected)
    % Obstacle definitions              => Use objects.txt object definition or default
  end
  
  methods
    %% constructor
    % DONE:
    % *Inputs:*
    % see properties for explanation of inputs
    function self = MpcPlanner(x0, xF, N, N_opt, Ts, lOb, tt, TEB_list, tau)
      self.x0 = x0;
      self.xF = xF;
      self.N = N;
      self.N_opt = N_opt;
      self.Ts = Ts;
      self.lOb = lOb;
      self.tt = tt;
      self.TEB = TEB_list;
      self.tau = tau;
    end
        
    %% Run
    % Main MPC path planning algorithm
    function Run(self)
      
        % Solve point-to-point OCP for initialization
        [self.stateP2P, self.controlP2P, self.runtimeP2P] = ...
            p2pWS(self.x0,self.xF,self.N,self.Ts);

        % Iterative warmstart
        stateWS = self.stateP2P;
        controlWS = self.controlP2P;
        self.runtimeIteWS = 0;        
        %   Start Iteration
        for scaleFactor = self.ScaleRange
            %   Scale Obstacles
            lObNew = scaleObs(length(self.nOb),self.nOb,self.lOb,scaleFactor);
            %   obtain H-rep of obstacles
            [AObNew, bObNew] = obsHrep(length(self.nOb), self.nOb, lObNew);
            %   Run MPC
            fprintf('**** Iterative Warm Start at Scaling Factor %f ****\n',scaleFactor)
            [stateOut, controlOut, solverTime, out] = ...
                IteWS(self.x0,self.xF,self.N,self.Ts,self.TEB,stateWS,controlWS,AObNew,bObNew,self.dmin);
            if out.CONVERGENCE_ACHIEVED==1
                fprintf('**** Problem solved SUCCESSFULLY ****\n')
                stateWS = stateOut;
                controlWS = controlOut;
                self.stateIteWS{end+1} = stateOut;
                self.controlIteWS{end+1} = controlWS;
                delete(self.hSO);
                delete(self.hST);
                [self.hSO,self.hST] = plotScaled(stateOut,lObNew, [0.5 0.5 0.5], ':',true);
            else
%                 delete(self.hSO);
%                 delete(self.hST);
                
                x0_relax = [self.x0(1);0.3*self.x0(2);self.x0(3);0.3*self.x0(4)];
                lObNew = scaleObs(length(self.nOb),self.nOb,self.lOb,0.5*scaleFactor);
                [AObNew, bObNew] = obsHrep(length(self.nOb), self.nOb, lObNew);
                [stateOut, controlOut, solverTime, out] = ...
                    IteWS(x0_relax,self.xF,self.N,self.Ts,self.TEB,stateWS,controlWS,AObNew,bObNew,self.dmin);
                stateWS = stateOut;
                controlWS = controlOut;
                self.stateIteWS{end+1} = stateOut;
                self.controlIteWS{end+1} = controlWS;
                delete(self.hSO);
                delete(self.hST);
                [self.hSO,self.hST] = plotScaled(stateOut,lObNew, [0.5 0.5 0.5], ':',true);
                
                fprintf('**** WARNING: Problem could not be solved ****\n')
            end
            self.runtimeIteWS = self.runtimeIteWS + solverTime;
        end
                
        % Final optimization
        flag = false;
        if self.isFinalOpt
            [AOb, bOb] = obsHrep(length(self.nOb), self.nOb, self.lOb);
            [self.stateOpt, self.controlOpt, self.runtimeOpt, out] = ...
                FinalOpt(self.x0,self.xF,self.N_opt,self.Ts,...
                self.tt,self.TEB,self.tau,self.stateIteWS{end},...
                self.controlIteWS{end},AOb,bOb,self.dmin);
            flag = out.CONVERGENCE_ACHIEVED;
        end
        
        % Visualization
        if flag
            fprintf('**** Final OPT solved SUCCESSFULLY ****\n')
            delete(self.hSO);
            delete(self.hST);
            [~,self.hT] = plotScaled(self.stateOpt, self.lOb, [0.5 0.5 0.5], ':',false);
        else
            if self.isFinalOpt
                fprintf('**** WARNING: Final OPT could not be solved ****\n')
                % ***
                self.runtimeOpt = 0;
            else
                fprintf('**** NO Final OPT, using warmstart path ****\n')
            end
            delete(self.hSO);
            delete(self.hST);
            [~,self.hT] = plotScaled(self.stateIteWS{end}, self.lOb, [0.5 0.5 0.5], ':',false);
            self.stateOpt = self.stateIteWS{end};
            self.controlOpt = self.controlIteWS{end};
        end      
              
    end
    
    %% Delete last plot 
    function delete(self)
      try delete(self.figure_h);end %#ok<TRYNC>
    end
    
  end
  
end