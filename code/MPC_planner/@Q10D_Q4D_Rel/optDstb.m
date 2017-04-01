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
  % Planning control
  if any(dims == 2)
    dOpt{2} = ((-deriv{dims==2})>=0)*(obj.aMax(1)) + ...
              ((-deriv{dims==2})<0)*(obj.aMin(1));
  end
  
  if any(dims == 6)
    dOpt{4} = ((-deriv{dims==6})>=0)*(obj.aMax(2)) + ...
              ((-deriv{dims==6})<0)*(obj.aMin(2));
  end  
  
  % Disturbances
  if any(dims == 1)
    dOpt{1} = ((deriv{dims==1})>=0)*(obj.dMax(1)) + ...
              ((deriv{dims==1})<0)*(obj.dMin(1));
  end
  
  if any(dims == 5)
    dOpt{3} = ((deriv{dims==5})>=0)*(obj.dMax(2)) + ...
              ((deriv{dims==5})<0)*(obj.dMin(2));
  end
  
elseif strcmp(dMode, 'min')
  % Planning control
  if any(dims == 2)
    dOpt{2} = ((-deriv{dims==2})>=0)*(obj.aMin(1)) + ...
              ((-deriv{dims==2})<0)*(obj.aMax(1));
  end
  
  if any(dims == 6)
    dOpt{4} = ((-deriv{dims==6})>=0)*(obj.aMin(2)) + ...
              ((-deriv{dims==6})<0)*(obj.aMax(2));
  end  
  
  % Disturbances
  if any(dims == 1)
    dOpt{1} = ((deriv{dims==1})>=0)*(obj.dMin(1)) + ...
              ((deriv{dims==1})<0)*(obj.dMax(1));
  end
  
  if any(dims == 5)
    dOpt{3} = ((deriv{dims==5})>=0)*(obj.dMin(2)) + ...
              ((deriv{dims==5})<0)*(obj.dMax(2));
  end
  
else
  error('Unknown uMode!')
end

if convert_back
  dOpt = cell2mat(dOpt);
end
end