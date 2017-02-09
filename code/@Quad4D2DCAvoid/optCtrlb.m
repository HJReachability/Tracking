function bOpt = optCtrlb(obj, ~, ~, deriv, bMode, ~)
% uOpt = optCtrl(obj, t, y, deriv, uMode, dims)

%% Input processing
if nargin < 5
  bMode = 'min';
end

convert_back = false;
if ~iscell(deriv)
  convert_back = true;
  deriv = num2cell(deriv);
end

dims = obj.dims;
%% Optimal control
if strcmp(bMode, 'max')
  if any(dims == 1)
    bOpt{1} = (-deriv{dims==1}>=0)*obj.bMax(1) + ...
      (-deriv{dims==1}<0)*(obj.bMin(1));
  end
  
  if any(dims == 3)
    bOpt{2} = (-deriv{dims==3}>=0)*obj.bMax(2) + ...
    (-deriv{dims==3}<0)*(obj.bMin(2));
  end
  
elseif strcmp(bMode, 'min')


else
    error('Unknown bMode!')
end

if convert_back
  bOpt = cell2mat(bOpt);
end
end