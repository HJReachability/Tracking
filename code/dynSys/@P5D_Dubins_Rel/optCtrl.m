function uOpt = optCtrl(obj, t, y, deriv, uMode, ~)
% uOpt = optCtrl(obj, t, y, deriv, uMode, ~)

if nargin < 5
  uMode = 'max';
end

if ~(strcmp(uMode, 'max') || strcmp(uMode, 'min'))
  error('uMode must be ''max'' or ''min''!')
end

if ~iscell(y)
  deriv = num2cell(y);
end

if ~iscell(deriv)
  deriv = num2cell(deriv);
end

uOpt = cell(obj.nu, 1);

%% Optimal control
if strcmp(uMode, 'max')
  if any(obj.dims == 4)
    uOpt{1} = (deriv{obj.dims==4}>=0)*obj.aRange(2) + ...
      (deriv{obj.dims==4}<0)*obj.aRange(1);
  end
  
  if any(obj.dims == 5)
    uOpt{2} = (deriv{obj.dims==5}>=0)*(obj.alphaMax) + ...
      (deriv{obj.dims==5}<0)*(-obj.alphaMax);
  end

elseif strcmp(uMode, 'min')
  if any(obj.dims == 4)
    uOpt{1} = (deriv{obj.dims==4}>=0)*obj.aRange(1) + ...
      (deriv{obj.dims==4}<0)*obj.aRange(2);
  end
  
  if any(obj.dims == 5)
    uOpt{2} = (deriv{obj.dims==5}>=0)*(-obj.alphaMax) + ...
      (deriv{obj.dims==5}<0)*(obj.alphaMax);
  end
else
  error('Unknown uMode!')
end
end