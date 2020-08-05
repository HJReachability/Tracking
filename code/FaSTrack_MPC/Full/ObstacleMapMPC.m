classdef ObstacleMapMPC < handle
% Obstacle map class for MPC
  
  properties
    % 2D Obtacles
    global_obs
    local_obs
    padded_obs
    vOb
    nObs % Number of obstacles
    
    % handles for plots
    hG
    hL
    hP
    hS
  end
  
  methods
    %% Constructor
    function self = ObstacleMapMPC(vOb,resolution)
      self.global_obs = initObs(vOb,resolution);
      self.nObs = length(self.global_obs);    
      self.local_obs = cell(1,self.nObs);
      self.padded_obs = cell(1,self.nObs);
      self.vOb = [];      
    end
    
    %% SenseAndUpdate
    % sense for obstacles that are within sense_range of x0
    % then update local and padded obstacles.
    function sense_update(self, x0, sense_radius, track_err)
        self.local_obs = updateLocal(x0, self.global_obs, self.local_obs, sense_radius);
        self.padded_obs = updatePadded(self.local_obs, track_err);
        self.vOb = padded2vOb(self.padded_obs);
        self.checkOverlap;
    end
    
    %% plotGlobal
    function plotGlobal(self, color, linestyle)
      if nargin < 2
        color = 'k';
      end
      
      if nargin < 3
        linestyle = ':';
      end
      
      extraArgs.LineStyle = linestyle;
      self.hG = plotGlobal(self.global_obs, color, linestyle, extraArgs);
    end
    
    %% plotLocal
    function plotLocal(self, color, linestyle)
      if nargin < 2
        color = 'r';
      end
      
      if nargin < 3
        linestyle = '-';
      end
      
      delete(self.hL)
      
      extraArgs.LineStyle = linestyle;
      self.hL = plotLocal(self.local_obs, color, linestyle, extraArgs);
    end
    
    %% plotPadded
    function plotPadded(self, color, linestyle)
      if nargin < 2
        color = 'b';
      end
      
      if nargin < 3
        linestyle = '--';
      end
      
      delete(self.hP)
      
      extraArgs.LineStyle = linestyle;
      self.hP = plotPadded(self.padded_obs, color, linestyle, extraArgs);
    end
    
    %% plotSenseRegion
    function plotSenseRegion(self, x0, senseRadius, varargin)
      if nargin < 3
        senseRadius = 2.0;
      end  
        
      if nargin < 4
        varargin = 'g-';
      end
      
      delete(self.hS)
      
      self.hS = plotDisk(x0, senseRadius, varargin);
    end
    
    %% checkOverlap
    % Detect obstacle overlap and merge them if so
    function checkOverlap(self)
        flag = false;
        lOb = getlOb([self.vOb{1,1};self.vOb{1,2};self.vOb{1,3};self.vOb{1,4}],...
                     [self.vOb{2,1};self.vOb{2,2};self.vOb{2,3};self.vOb{2,4}],...
                     [self.vOb{3,1};self.vOb{3,2};self.vOb{3,3};self.vOb{3,4}]);
        for i = 1 : self.nObs	
            V = [];
            for l = 1 : 4
                V = [V ; [lOb{i,l}(1), lOb{i,l}(2)] ];
            end
            poly = Polyhedron('V',V);
            for j = 1 : self.nObs
                if j == i
                    continue
                else
                    for k = 1:4
                        if poly.contains(lOb{j,k}) && norm(lOb{j,k}) < 100                            
                            self.padded_obs = mergePadded(self.padded_obs,i,j);
                            self.vOb = padded2vOb(self.padded_obs);
                            lOb = getlOb([self.vOb{1,1};self.vOb{1,2};self.vOb{1,3};self.vOb{1,4}],...
                                         [self.vOb{2,1};self.vOb{2,2};self.vOb{2,3};self.vOb{2,4}],...
                                         [self.vOb{3,1};self.vOb{3,2};self.vOb{3,3};self.vOb{3,4}]);
                            flag = true;
%                             fprintf('\n******************************************\n');
%                             fprintf('***     Obstacle overlap detected      ***\n');
%                             fprintf('******************************************\n');                        
                        end
                        if flag
                            break
                        end                               
                    end
                end
            end
        end        
%         % Recheck
%         if flag
%             checkOverlap(self)
%         end
    end
    
  % END OF METHODS
  end
end