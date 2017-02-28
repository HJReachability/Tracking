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
  if any(dims == 1)
      %player 1 tries to min
    uOpt{1} = (-deriv{dims==1}>=0)*(obj.uMin(1)) + ...
      (-deriv{dims==1}<0)*obj.uMax(1);
  end

  if any(dims == 2)
    %player 2 tries to max
    uOpt{2} = (deriv{dims==2}>=0)*obj.uMax(2) + ...
      (deriv{dims==2}<0)*(obj.uMin(2));
  end
  
    
  if any(dims == 3)
    %player 1 tries to min
    uOpt{3} = (-deriv{dims==3}>=0)*(obj.uMin(3)) + ...
      (-deriv{dims==3}<0)*obj.uMax(3);
  end
  
  
  if any(dims == 4)
     %player 2 tries to max
    uOpt{4} = (deriv{dims==4}>=0)*obj.uMax(4) + ...
    (deriv{dims==4}<0)*(obj.uMin(4));
  end
  
elseif strcmp(uMode, 'min')
  if any(dims == 1)
      %player 1 tries to max
    uOpt{1} = (-deriv{dims==1}>=0)*obj.uMax(1) + ...
      (-deriv{dims==1}<0)*(obj.uMin(1));
  end
 
  if any(dims == 2)
    %player 2 tries to min
    uOpt{2} = (deriv{dims==2}>=0)*(obj.uMin(2)) + ...
      (deriv{dims==2}<0)*obj.uMax(2);
  end
    
  if any(dims == 3)
    %player 1 tries to max
    uOpt{3} = (-deriv{dims==3}>=0)*obj.uMax(3) + ...
    (-deriv{dims==3}<0)*(obj.uMin(3));
  end
  
  if any(dims == 4)
    %player 2 tries to min
    uOpt{4} = (deriv{dims==4}>=0)*(obj.uMin(4)) + ...
      (deriv{dims==4}<0)*obj.uMax(4);
  end

else
    error('Unknown uMode!')
end

if convert_back
  uOpt = cell2mat(uOpt);
end
end