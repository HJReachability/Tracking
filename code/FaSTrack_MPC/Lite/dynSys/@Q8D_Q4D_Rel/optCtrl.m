function uOpt = optCtrl(obj, ~, ~, deriv, uMode, ~)
% uOpt = optCtrl(obj, t, y, deriv, uMode, dims)

%% Input processing
if nargin < 5
  uMode = 'max';
end

convert_back = false;
if ~iscell(deriv)
  convert_back = true;
  deriv = num2cell(deriv);
end

dims = obj.dims;
%% Optimal control
if strcmp(uMode, 'max')
  if any(dims == 4)
    uOpt{1} = (deriv{dims==4}>=0)*obj.uMax(2) + (deriv{dims==4}<0)*obj.uMin(1);
  end
  
  if any(dims == 8)
    uOpt{2} = (deriv{dims==8}>=0)*obj.uMax(4) + (deriv{dims==8}<0)*obj.uMin(2);
  end
  
elseif strcmp(uMode, 'min')
  if any(dims == 4)
    uOpt{1} = (deriv{dims==4}<0)*obj.uMax(2) + (deriv{dims==4}>=0)*obj.uMin(1);
  end
  
  if any(dims == 8)
    uOpt{2} = (deriv{dims==8}<0)*obj.uMax(4) + (deriv{dims==8}>=0)*obj.uMin(2);
  end
else
  error('Unknown uMode!')
end

if convert_back
  uOpt = cell2mat(uOpt);
end
end