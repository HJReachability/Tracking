function error = cpp_mat_compare(gCPP, dataCPP, dataMAT)


if nargin > 2
  error.mean = zeros(length(dataCPP),1);
  error.max = zeros(length(dataCPP),1);
  
  
  for i = 1:length(dataCPP)
    dataMAT_i = dataMAT(:,:,:,:,:,i);
    dataError_i = abs(dataCPP{i}(:) - dataMAT_i(:));
    error.mean(i) = mean(dataError_i);
    error.max(i) = max(dataError_i);
    
    max_dataMAT = max(dataMAT_i(:));
    min_dataMAT = min(dataMAT_i(:));
    
    mag = max_dataMAT - min_dataMAT;
    error.mean_rel = error.mean/mag;
    error.max_rel = error.max/mag;
  end
end

level = -0.1;
g = cpp2matG(gCPP);
[g3D_cpp, data3D_cpp] = proj(g, dataCPP, [0 0 0 1 1], [0 0]);

figure;
visSetIm(g3D_cpp, data3D_cpp, 'r', level);

if nargin > 2
  [g3D_MAT, data3D_MAT] = proj(g, dataCPP, [0 0 0 1 1], [0 0]);
  
  visSetIm(g3D_MAT, data3D_MAT, 'b', level);
end

end