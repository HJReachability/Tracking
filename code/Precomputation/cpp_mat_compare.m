function error = cpp_mat_compare(dataCPP, dataMAT)

error.mean = zeros(length(dataCPP),1);
error.max = zeros(length(dataCPP),1);

for i = 1:length(dataCPP)
  dataError_i = abs(dataCPP{i} - dataMAT(:,:,:,:,:,i));
  error.mean(i) = mean(dataError_i(:));
  error.max(i) = max(dataError_i(:));
end

end