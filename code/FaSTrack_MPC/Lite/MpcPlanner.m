%% MpcPlanner (Ver. Lite)
classdef MpcPlanner < handle
  % *Description:* MAIN FUNCTION: This function is a basic implementation of
  % the model predictive control (MPC) path planning algorithm with
  % optimization-based collision avoidance approach [Zhang et al. 2017]
  %
  % *Author:* Haimin Hu
  %
  % *Last Updated:* 3rd August 2020
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
    function Run(self, mpc, iter)
        
        % Visualization
        if iter <= 45
            dark_cyan = [0.1686 0.5843 0.6627];
            color = dark_cyan;
        elseif 45<iter && iter<=75
            color = 'b';
        else
            color = 'm';
        end
        
        grey = [0.5 0.5 0.5];
        
        delete(self.hSO);
        delete(self.hST);
        [~,self.hT] = plotScaled(mpc.stateOpt, self.lOb, color, ':',false);
        
    end
    
    %% Delete last plot 
    function delete(self)
      try delete(self.figure_h);end %#ok<TRYNC>
    end
    
  end
  
end