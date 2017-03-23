%Test script
world = createWorld;

x1 = 1:100; x2 = 1:100;

figure
surf(x1,x2,world);
caxis([min(world(:))-.5*range(world(:)),max(world(:))]);
axis([0 100 0 100 0 1])
xlabel('x1'); ylabel('x2'); zlabel('Probability Density');
hold on;

%Init class
BlueRov = AUV;
BlueRov.sparseTraverse(10,world,'E','S')
z = ones(size(BlueRov.previous_x)) * 0.3;
plot3(BlueRov.previous_x, BlueRov.previous_y,z,'r');
hold off;

