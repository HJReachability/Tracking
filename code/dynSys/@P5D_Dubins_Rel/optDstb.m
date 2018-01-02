function dOpt = optDstb(obj, ~, y, deriv, dMode, ~)
% dOpt = optDstb(obj, t, y, deriv, ~, ~)

if nargin < 5
  dMode = 'max';
end

if ~(strcmp(dMode, 'max') || strcmp(dMode, 'min'))
  error('dMode must be ''max'' or ''min''!')
end

if ~iscell(y)
  y = num2cell(y);
end

if ~iscell(deriv)
  deriv = num2cell(deriv);
end

%% Optimal planning control
% Determinant for sign of control
det = deriv{1}.*y{2}  - deriv{2}.*y{1} - deriv{3};

% Maximize Hamiltonian
if strcmp(dMode, 'max')
  dOpt{5} = (det>=0)*obj.wMax + (det<0)*(-obj.wMax);
else
  dOpt{5} = (det>=0)*(-obj.wMax) + (det<0)*obj.wMax;
end

%% Optimal disturbance
if strcmp(dMode, 'max')
  if any(obj.dims == 1)
    dOpt{1} = (deriv{obj.dims==1}>=0)*obj.dMax(1) + ...
      (deriv{obj.dims==1}<0)*(-obj.dMax(1));
  end
  
  if any(obj.dims == 2)
    dOpt{2} = (deriv{obj.dims==2}>=0)*obj.dMax(2) + ...
      (deriv{obj.dims==2}<0)*(-obj.dMax(2));
  end
  
  if any(obj.dims == 4)
    dOpt{3} = (deriv{obj.dims==3}>=0)*obj.dMax(3) + ...
      (deriv{obj.dims==3}<0)*(-obj.dMax(3));
  end
  
  if any(obj.dims == 5)
    dOpt{4} = (deriv{obj.dims==4}>=0)*obj.dMax(4) + ...
      (deriv{obj.dims==4}<0)*(-obj.dMax(4));
  end  

elseif strcmp(dMode, 'min')
  if any(obj.dims == 1)
    dOpt{1} = (deriv{obj.dims==1}>=0)*(-obj.dMax(1)) + ...
      (deriv{obj.dims==1}<0)*obj.dMax(1);
  end
  
  if any(obj.dims == 2)
    dOpt{2} = (deriv{obj.dims==2}>=0)*(-obj.dMax(2)) + ...
      (deriv{obj.dims==2}<0)*obj.dMax(2);
  end
  
  if any(obj.dims == 4)
    dOpt{3} = (deriv{obj.dims==3}>=0)*(-obj.dMax(3)) + ...
      (deriv{obj.dims==3}<0)*obj.dMax(3);
  end
  
  if any(obj.dims == 5)
    dOpt{4} = (deriv{obj.dims==4}>=0)*(-obj.dMax(4)) + ...
      (deriv{obj.dims==4}<0)*obj.dMax(4);
  end  
  
else
  error('Unknown dMode!')
end

end