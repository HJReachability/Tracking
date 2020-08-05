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
uOpt = cell(3,1);
%% Optimal control
if strcmp(uMode, 'max')
    for ii = 1:3
        uOpt{ii} = (deriv{dims==ii}>=0)*obj.uMax(ii) + (deriv{dims==ii}<0)*obj.uMin(ii);
    end
    
elseif strcmp(uMode, 'min')
    for ii = 1:3
        uOpt{ii} = (deriv{dims==ii}<0)*obj.uMax(ii) + (deriv{dims==ii}>=0)*obj.uMin(ii);
    end
else
  error('Unknown uMode!')
end

if convert_back
  uOpt = cell2mat(uOpt);
end
end