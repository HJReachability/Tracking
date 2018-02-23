function Q8D_Q4D_gti_test()

% Grid
N = 25;
g = createGrid(-2*ones(4,1), 2*ones(4,1), N*ones(4,1));

% Example value function and tracking error bound
tau_length = 20;
vf = zeros([N*ones(1,4) tau_length]);

TEB = zeros(1, tau_length);
level = -2;

dist = sqrt(g.xs{1}.^2 + g.xs{2}.^2 + g.xs{3}.^2 + g.xs{4}.^2);
for i = 1:tau_length
  if i > 10 && i < 15
    vf(:,:,:,:,i) = -dist - 1;
    TEB(i) = -level - 1;
  else
    vf(:,:,:,:,i) = -dist - 0.1*i;
    TEB(i) = -level - 0.1*i;
  end
end

% Call get tracking error bound index function

s = -1 + 2*rand(8,1);
p = -1 + 2*rand(4,1);

if rand > 0.5
  s = s / 10;
  p = p / 10;
end

[indX, indY, TEB_list] = Q8D_Q4D_gti(s, p, g, vf, level, TEB);

% Check results
r = Q8D_Q4D_grs(s, p);

vx = eval_u(g, vf(:,:,:,:,indX), r(1:4));
vy = eval_u(g, vf(:,:,:,:,indY), r(5:8));
fprintf('Values at indices (%d, %d): (%.2f, %.2f).\n', indX, indY, vx ,vy)

if indX + 1 <= tau_length
  vx_next = eval_u(g, vf(:,:,:,:,indX+1), r(1:4));
else
  vx_next = -100;
end

if indY + 1 <= tau_length
  vy_next = eval_u(g, vf(:,:,:,:,indY+1), r(5:8));
else
  vy_next = -100;
end

fprintf('Values at indices (%d, %d): (%.2f, %.2f).\n', ...
  indX+1, indY+1, vx_next, vy_next)

if min([indX, indY])-1 > 0
  fprintf('TEB_list should start from %.2f.\n', TEB(min([indX indY])-1))
else
  fprintf('TEB_list should be empty.\n')
end
disp('TEB_list:')
disp(TEB_list)

end