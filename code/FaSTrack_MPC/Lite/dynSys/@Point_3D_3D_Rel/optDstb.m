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

dOpt = cell(3,1);
dims = obj.dims;
%% Optimal disturbance
if strcmp(dMode, 'max')
    for ii = 1:3
        dOpt{ii} = (-deriv{dims==ii}>=0)*obj.pMax(ii) + (-deriv{dims==ii}<0)*obj.pMin(ii);
    end
    
elseif strcmp(dMode, 'min')
    for ii = 1:3
        dOpt{ii} = (-deriv{dims==ii}<0)*obj.pMax(ii) + (-deriv{dims==ii}>=0)*obj.pMin(ii);
    end
else
  error('Unknown dMode!')
end
  

if convert_back
  dOpt = cell2mat(dOpt);
end
end