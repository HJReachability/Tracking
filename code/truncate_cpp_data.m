function [g, data] = truncate_cpp_data(gCPP, dataCPP, xmin, xmax, process)

if nargin < 5
  process = true;
end

gOld = cpp2matG(gCPP);
g = truncateGrid(gOld, [], xmin, xmax, process);

numT = length(dataCPP);
data = zeros([g.N' numT]);

clns = repmat({':'}, 1, gOld.dim);
for i = 1:numT
  [~, data(clns{:}, i)] = truncateGrid(gOld, dataCPP{i}, xmin, xmax, process);
end

end