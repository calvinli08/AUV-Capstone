%Test script
world = createWorld;

x1 = 1:100; x2 = 1:100;

figure(1)
surf(x1,x2,world);
%caxis([min(world(:))-.5*range(world(:)),max(world(:))]);
axis([0 100 0 100 0 1])
xlabel('x1'); ylabel('x2'); zlabel('Probability Density');
hold on;


%% Phase 1 Test - Full algo
%Init class
BlueRov = AUV;
Densetraverse = [2,5,5];
BlueRov.sparseTraverse(10,[0.3,0.7],world,'E','S',Densetraverse)
z = ones(size(BlueRov.previous_x)) * 0.3;
plot3(BlueRov.previous_x, BlueRov.previous_y,z,'r');
hold off;

%% Phase 2 Test - Gather POIs
figure
scatter(BlueRov.pollution_sources(1,:),BlueRov.pollution_sources(2,:) )
title('Pollution Sources');



%% Phase 2 Test - Trimming POIs
tt = trim(transpose(BlueRov.points_of_interest),3)
figure
scatter(tt(:,1),tt(:,2))
title('Points of Interest');

%% Phase 2 Test - Gradients
figure
[px,py] = gradient(world);
quiver(px,py);
title('World Gradients');
xlim([0,100]);
ylim([0,100]);

figure
[ppx,ppy] = gradient(BlueRov.current_knowledge);
quiver(ppx,ppy);
title('Sparse Traversal Gradients');
xlim([0,100]);
ylim([0,100]);

%% Phase 2 Test - Point to Point
PointtoPoint = AUV;
PointtoPoint.point_to_point(50,50);
z = ones(size(PointtoPoint.previous_x)) * 0.3;
figure
plot3(PointtoPoint.previous_x, PointtoPoint.previous_y,z,'r');

%% Phase 2 Test - Nearest Neighbor
P = [1;1]
X = [1, 2, 3; 2, 2, 2];
I = nearestneighbour(P,X);
next = X(:,I);
next(1)
next(2)
disp(X(:,I)); %Expected 1,2
%Nice it works



%% Phase 2 Test - Square(Dense) Traverse Test
SquareTrav = AUV;
SquareTrav.point_to_point(50,50);
SquareTrav.squareTraverse(0,4,world);
z = ones(size(SquareTrav.previous_x)) * 0.3;
figure
plot(SquareTrav.previous_x, SquareTrav.previous_y,'r');
grid on;

%% Phase 2 Test 
DenseAUV = AUV;
DenseAUV.point_to_point(50,50);
DenseAUV.denseTraverse(world,2,10,5,'E','S')
figure
plot(DenseAUV.previous_x, DenseAUV.previous_y,'r');
grid on;

%% 