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
deriv_uOpt1 = deriv{1}.*x{2}-deriv{2}.*x{1};

if strcmp(uMode, 'max')
    
  uOpt{1} = (deriv_uOpt1>=0)*obj.wMax +...
      (deriv_uOpt1<0)*obj.wMin;
  
  uOpt{2} = (deriv{3}>=0)*obj.aMax +...
      (deriv{3}<0)*obj.aMin;
  
elseif strcmp(uMode, 'min')
    uOpt{1} = (deriv_uOpt1>=0)*obj.wMin +...
      (deriv_uOpt1<0)*obj.wMax;
  
  uOpt{2} = (deriv{3}>=0)*obj.aMin +...
      (deriv{3}<0)*obj.aMax;
else
  error('Unknown uMode!')
end

if convert_back
  uOpt = cell2mat(uOpt);
end
end