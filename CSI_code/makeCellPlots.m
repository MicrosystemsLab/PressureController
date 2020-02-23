% makeCellPlots
%load('waterImTSeries.mat')
load('cellAggregate.mat')
figure; plot(cellAreaAvg,'o-'); hold on
%errorbar(cellAreaAvg,cellAreaStd)
grid on
title('Average Cell Area')
ylabel('Cell Area [pixel^2]')
xlabel('Pressure Step')

areapct = (cellAreaAvg - cellAreaAvg(1))./cellAreaAvg(1) .* 100;

figure; plot(areapct,'o-')
grid on
title('Average Cell Area')
ylabel('Cell Area % Change')
xlabel('Pressure Step')

figure; plot(cellPerimeterAvg,'o-'); hold on
%errorbar(cellPerimeterAvg,cellPerimeterStd)
grid on
title('Average Cell Perimeter')
ylabel('Cell Perimeter [pixel]')
xlabel('Pressure Step')

permpct = (cellPerimeterAvg - cellPerimeterAvg(1))./cellPerimeterAvg(1) .* 100;

figure; plot(permpct,'o-')
grid on
title('Average Cell Perimeter')
ylabel('Cell Perimeter % Change')
xlabel('Pressure Step')

figure; plot(cellEccentricityAvg,'o-'); hold on
%errorbar(cellEccentricityAvg,cellEccentricityStd)
title('Cell Eccentricity')
grid on
