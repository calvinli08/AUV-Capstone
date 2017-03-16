world = rand(100,100);
world = .3 .* world;

for n = 1:6 % create six regions to add into the world.
    a = genInteres(5);
    b = randi(89,1);
    c = randi(89,1);
    world(b:b+10,c: c+10) = a;

end

x1 = 1:100; x2 = 1:100;

surf(x1,x2,world);
caxis([min(world(:))-.5*range(world(:)),max(world(:))]);
axis([0 100 0 100 0 1])
xlabel('x1'); ylabel('x2'); zlabel('Probability Density');