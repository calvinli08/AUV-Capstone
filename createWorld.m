function[world] = createWorld()
world = rand(100,100);
world = .3 .* world;

for n = 1:6 % create six regions to add into the world.
    a = genInteres(5);
    b = randi(89,1);
    c = randi(89,1);
    world(b:b+10,c: c+10) = a;

end
