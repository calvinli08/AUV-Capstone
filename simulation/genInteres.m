function [F] = genInteres(a)
a  = 5;
mu = [0 0];
Sigma = [.25 .3; .3 1];
x1 = -a:1:a; x2 = -a:1:a;
[X1,X2] = meshgrid(x1,x2);
F = mvnpdf([X1(:) X2(:)],mu,Sigma);
F = reshape(F,length(x2),length(x1)); 
F = F ./max(max(F));
%surf(x1,x2,F);
%caxis([min(F(:))-.5*range(F(:)),max(F(:))]);
%axis([-3 3 -3 3 0 1])
%xlabel('x1'); ylabel('x2'); zlabel('Probability Density');
end