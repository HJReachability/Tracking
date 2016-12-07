function pl_track_pl_vis()
load('pl_track_pl/set.mat')

figure
visSetIm(g, data, 'r', -0.2);

figure
colors = hsv(g.N(3));
for i = 1:g.N(3)
  [g2D, data2D] = proj(g, data, [0 0 1], g.vs{3}(i));
  visSetIm(g2D, data2D, colors(i,:), -0.2);
  hold on
end
end
