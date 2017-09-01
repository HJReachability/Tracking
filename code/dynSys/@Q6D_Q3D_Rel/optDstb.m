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
  if any(dims == 1)
    dOpt{1} = ((-deriv{dims==1})>=0)*(obj.dMax(1)) + ...
      ((-deriv{dims==1})<0)*(obj.dMin(1));
  
    dOpt{2} = ((-deriv{dims==1})>=0)*(obj.pMax(1)) + ...
      ((-deriv{dims==1})<0)*(obj.pMin(1));
  end
  
  if any(dims == 2)
    dOpt{3} = ((-deriv{dims==2})>=0)*(obj.dMax(2)) + ...
      ((-deriv{dims==2})<0)*(obj.dMin(2));
  
    dOpt{4} = ((-deriv{dims==2})>=0)*(obj.pMax(2)) + ...
      ((-deriv{dims==2})<0)*(obj.pMin(2));
  end
  
  if any(dims == 3)
    dOpt{5} = ((-deriv{dims==3})>=0)*(obj.dMax(3)) + ...
      ((-deriv{dims==3})<0)*(obj.dMin(3));

    dOpt{6} = ((-deriv{dims==3})>=0)*(obj.pMax(3)) + ...
      ((-deriv{dims==3})<0)*(obj.pMin(3));
  end
elseif strcmp(dMode, 'min')
  if any(dims == 1)
    dOpt{1} = ((-deriv{dims==1})>=0)*(obj.dMin(1)) + ...
      ((-deriv{dims==1})<0)*(obj.dMax(1));
  
    dOpt{2} = ((-deriv{dims==1})>=0)*(obj.pMin(1)) + ...
      ((-deriv{dims==1})<0)*(obj.pMax(1));
  end
  
  if any(dims == 2)
    dOpt{3} = ((-deriv{dims==2})>=0)*(obj.dMin(2)) + ...
      ((-deriv{dims==2})<0)*(obj.dMax(2));
  
    dOpt{4} = ((-deriv{dims==2})>=0)*(obj.pMin(2)) + ...
      ((-deriv{dims==2})<0)*(obj.pMax(2));
  end
  
  if any(dims == 3)
    dOpt{5} = ((-deriv{dims==3})>=0)*(obj.dMin(3)) + ...
      ((-deriv{dims==3})<0)*(obj.dMax(3));

    dOpt{6} = ((-deriv{dims==3})>=0)*(obj.pMin(3)) + ...
      ((-deriv{dims==3})<0)*(obj.pMax(3));
  end
  
else
  error('Unknown dMode!')
end

if convert_back
  dOpt = cell2mat(dOpt);
end
end