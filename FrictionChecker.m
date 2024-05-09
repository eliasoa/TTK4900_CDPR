velt = linspace(-5,5,1000);
balle = zeros(size(velt,2),1);
for i=1:size(velt,2)
    balle(i) = FrictionModel(1,velt(i));
end


figure(51)
plot(velt, balle)

% figure(50)
% plot([0,0]); hold on;
% plot([1,1])
% plot([2,2])
% plot([3,3]); hold off;