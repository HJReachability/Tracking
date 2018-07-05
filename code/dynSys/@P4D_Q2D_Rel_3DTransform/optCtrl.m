function uOpt = optCtrl(obj, ~, x, deriv, uMode, ~)
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
  uOpt{1} = ((-deriv{1}.*x{2}+deriv{2}.*x{1})>=0)*obj.uMax(1) +...
      ((-deriv{1}.*x{2}+deriv{2}.*x{1})<0)*obj.uMin(1);
  uOpt{2} = (deriv{3}>=0)*obj.uMax(2) +...
      (deriv{3}<0)*obj.uMin(2);
elseif strcmp(uMode, 'min')
    uOpt{1} = ((-deriv{1}.*x{2}+deriv{2}.*x{1})>=0)*obj.uMin(1) +...
      ((-deriv{1}.*x{2}+deriv{2}.*x{1})<0)*obj.uMax(1);
  uOpt{2} = (deriv{3}>=0)*obj.uMin(2) +...
      (deriv{3}<0)*obj.uMax(2);
else
  error('Unknown uMode!')
end

if convert_back
  uOpt = cell2mat(uOpt);
end
end