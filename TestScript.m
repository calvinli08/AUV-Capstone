%Test script
world = createWorld;

x1 = 1:100; x2 = 1:100;

figure
surf(x1,x2,world);
caxis([min(world(:))-.5*range(world(:)),max(world(:))]);
axis([0 100 0 100 0 1])
xlabel('x1'); ylabel('x2'); zlabel('Probability Density');
hold on;

%% Phase 1 Test - Sparse Traversal Function
%Init class
BlueRov = AUV;
BlueRov.sparseTraverse(10,world,'E','S')
z = ones(size(BlueRov.previous_x)) * 0.3;
plot3(BlueRov.previous_x, BlueRov.previous_y,z,'r');
hold off;


%% Phase 2 Test - Marking pollution levels
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