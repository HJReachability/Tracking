function [h, x, y] = plotDisk(center, radius, varargin)
% Plots a disk of some radius around a given center

    if radius < 0
      error('Radius must be non-negative!')
    end

    t = linspace(0, 2*pi, 100);
    x = radius*cos(t) + center(1);
    y = radius*sin(t) + center(2);

    h = [];
    if nargout < 2
      h = plot(x, y, varargin{:});
    end

    drawnow

end