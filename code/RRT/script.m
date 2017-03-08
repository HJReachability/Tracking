% obstacleFilename = 'obstacles.txt';
% seedsPerAxis = 7;
% treesMax = seedsPerAxis^3*3+2;
% rrt = RrtPlanner(treesMax,seedsPerAxis,obstacleFilename);
% 

origin = [1 2 3];
size = 3;
cube = [];
x=([0 1 1 0 0 0;1 1 0 0 1 1;1 1 0 0 1 1;0 1 1 0 0 0]-0.5)*size+origin(1);
y=([0 0 1 1 0 0;0 1 1 0 0 0;0 1 1 0 1 1;0 0 1 1 1 1]-0.5)*size+origin(2);
z=([0 0 0 0 0 1;0 0 0 0 0 1;1 1 1 1 0 1;1 1 1 1 0 1]-0.5)*size+origin(3);
for i=1:6
    h = patch(x(:,i),y(:,i),z(:,i),'w');
    cube(:, i) = h;
end 
