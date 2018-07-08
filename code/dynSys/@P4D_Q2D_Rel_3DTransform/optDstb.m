function dOpt = optDstb(obj, ~, ~, deriv, dMode, ~)
% uOpt = optCtrl(obj, t, y, deriv, uMode, dims)

%% Input processing
if nargin < 5
  dMode = 'min';
end

convert_back = false;
if ~iscell(deriv)
  convert_back = true;
  deriv = num2cell(deriv);
end

dims = obj.dims;
%% Optimal control
if strcmp(dMode, 'max')
    % Disturbances
    dOpt{1} = ((-deriv{dims==1})>=0)*(obj.dMax(1)) + ...
        ((-deriv{dims==1})<0)*(obj.dMin(1));
    
    % Disturbances
    dOpt{3} = ((-deriv{dims==2})>=0)*(obj.dMax(2)) + ...
        ((-deriv{dims==2})<0)*(obj.dMin(2));
    
    % Disturbances
    dOpt{5} = ((-deriv{dims==3})>=0)*(obj.dMax(3)) + ...
        ((-deriv{dims==3})<0)*(obj.dMin(3));
    
    
    %optimal planning control vector points along gradient. so
    %up* = (pMax)*deriv/||deriv|| where deriv = [deriv1, deriv2]
    
    denom = (deriv{1}.^2 + deriv{2}.^2 ).^(1/2);
    
    % Planning control
    dOpt{2} = (obj.pMax).*deriv{1}./denom;
    
    %if gradient was 0, replace those values with arbitrary control
    dOpt{2}(isnan(dOpt{2}))=((obj.pMax)^2/2).^2; 
    
    % Planning control
    dOpt{4} = (obj.pMax).*deriv{2}./denom;
    
    %if gradient was 0, replace those values with arbitrary control
    dOpt{4}(isnan(dOpt{4}))=((obj.pMax)^2/2).^2; 
    
elseif strcmp(dMode, 'min')

    % Disturbances
    dOpt{1} = ((-deriv{dims==1})>=0)*(obj.dMin(1)) + ...
              ((-deriv{dims==1})<0)*(obj.dMax(1));
            
    % Disturbances
    dOpt{3} = ((-deriv{dims==2})>=0)*(obj.dMin(2)) + ...
              ((-deriv{dims==2})<0)*(obj.dMax(2));
 
    % Disturbances
    dOpt{5} = ((-deriv{dims==3})>=0)*(obj.dMin(3)) + ...
              ((-deriv{dims==3})<0)*(obj.dMax(3));
          
    %optimal planning control vector points along opposite of gradient. so
    %up* = -(upmax)*p/||p|| where p = [deriv1, deriv2]
    
    denom = (deriv{1}.^2 + deriv{2}.^2 ).^(1/2);
    % Planning control
    dOpt{2} = -(obj.pMax).*deriv{1}./denom;
    
    % Planning control
    dOpt{4} = -(obj.pMax).*deriv{2}./denom;
else
  error('Unknown uMode!')
end

if convert_back
  dOpt = cell2mat(dOpt);
end
end