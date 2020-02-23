function watershedAvgIm(FNS,cellNperLength)
% WATERSHEDAVGIM(FNS,CELLNPERLENGTH) finds cell boundaries in a series of
%  microscope images of a cell monolayer.
% (CSI Step #2)
%

fprintf(1,'\nCSI/watershedAvgIm\n');

% load results from previous steps
load('tform.mat','tformCell')


%% load the fixed ("zero-strain") image
fixedAd = imgprocess2(FNS{1},1);

% determine allowed cell size
[m, n] = size(fixedAd);
fprintf(1,'CSI/watershedAvgIm: Image size is %d x %d pixels\n',m,n);
cellSize = (n*m)/(cellNperLength^2);
fprintf(1,'watershedAvgIm: using typical cell area of %.0f pixels^2\n',cellSize);
c_lower = cellSize / 2; % lower limit of cell size
c_upper = cellSize * 2.5;  % upper limit of cell size
fprintf(1,'CSI/watershedAvgIm: ignoring cells < %.0f and > %0.f pixels^2\n',c_lower,c_upper);



%% create "average image"
% Create an image that is the intensity average of all the images:
%  the "fixed image", and all the stretch images with the inverse spatial
%  transform applied. The resulting image is masked to the maximum area
%  that encompasses all of the images. This image will be used to find cell
%  boundaries.

% start average registered images
avgImg = fixedAd/size(FNS,2);
figure;

fprintf(1,'CSI/watershedAvgIm: read all images and average\n');
% loop over all images
for i = 2:size(FNS,2)
	% read the next image
    fprintf(1,' Image Transform: %d / %d: %s\n',i,size(FNS,2),FNS{i});
	% imgprocess2: apply image filtering and create single image (maximum
	%  intensity projection) from all images in z-stack
	movingAd = imgprocess2(FNS{i},1);
	
	% apply spatial transform to this image
	%  use a high fill value so we can mask it later
	tform = tformCell{i};
	movingRegistered = imwarp(movingAd,tform,'OutputView',imref2d(size(fixedAd)),'FillValues',10*size(FNS,2));

	imshow(movingRegistered);
	title(FNS{i},'Interpreter','none')
	drawnow;

	% add result to average registered image
	avgImg = avgImg + movingRegistered/size(FNS,2);
	
end

% close the figure showing the transform progress
pause(2); close;

% mask out regions that are not available in all images
avgMask = avgImg <= 1;
%figure; imshow(avgMask,'InitialMagnification','fit')
avgProp = regionprops(avgMask,'BoundingBox');
avgImg = imcrop(avgImg,avgProp.BoundingBox);
% avgImg(avgImg>=10) = 0;
figure; imshow(avgImg,'InitialMagnification',100)
title('CSI (2): Intensity Average from all Images','FontSize',14)


%% apply watershed algorithm to find cell boundaries

% Apply watershed algorithm to the average image
[waterIm, filteredIm] = applyWatershed(avgImg);

% Create binary mask of cell-cell contacts
bwIm = waterIm == false;
[m, n] = size(bwIm);

% Display results
bwIm = imdilate(bwIm, strel('sq', 2)); % make the lines thicker and easier to see
greenIm = cat(3, zeros(m, n), bwIm, zeros(m, n)); % create RGB image (with only G)
% boundaryIm = imfuse(filteredIm, greenIm, 'blend');
% rgb = label2rgb(waterIm,'jet',[.5 .5 .5]);

% save results
save('avgImg.mat','filteredIm','avgProp','greenIm','bwIm');


%% filter cells based on size
% WARNING: Uses magic numbers based on pixels/cell 
AreaIm = regionprops(waterIm,'Area','BoundingBox','Centroid');
Areas = cat(1,AreaIm.Area);
goodCells = [AreaIm.Area] >= c_lower & [AreaIm.Area] <= c_upper; % vector of booleans
ind = find([AreaIm.Area] >= c_lower & [AreaIm.Area] <= c_upper); % vector of indices
fprintf(1,'CSI/watershedAvgIm: identified %d cells out of %d possibilities\n',length(ind),length(AreaIm));

% show cell boundaries
figure %imshow(boundaryIm,'InitialMagnification',100)
imshow(filteredIm)
hold on
image('CData',greenIm,'AlphaData',bwIm)
title(sprintf('CSI (2): Cell Boundaries (%d regions)',length(AreaIm)),'FontSize',14)

Areas(Areas==max(Areas))=0; % eliminate masking artefact
c_mean = mean(Areas(ind));
c_std = std(Areas(ind));

% show size distribution
figure
histogram(Areas,round(length(ind)/2));
hold on
grid on
hcl(1) = plot(gca,[cellSize cellSize],ylim(gca),':m','LineWidth',2);
hcl(2) = plot(gca,[c_lower c_lower],ylim(gca),'-r');
hcl(3) = plot(gca,[c_upper c_upper],ylim(gca),'-r');
hcl(4) = plot(gca,[c_mean c_mean],ylim(gca),'-g');
hcl(5) = plot(gca,[c_mean-c_std c_mean-c_std],ylim(gca),'--b');
hcl(6) = plot(gca,[c_mean+c_std c_mean+c_std],ylim(gca),'--b');
%xlim([-1 3*c_upper]);
legend(hcl,{'Expected Cell Size','Cell Size Lower Limit','Cell Size Upper Limit',...
	'Cell Size Mean','Cell Size Mean-std.','Cell Size Mean+std'},'Location','northeast');
title('CSI (2): Cell Size Histogram','FontSize',12);
ylabel('Number of Cells');
xlabel('Cell Area [pixels^2]');
drawnow;

% show filtered cells
Iout = ismember(waterIm,ind);
badCell = ~ismember(waterIm,[ind 0]);
Iout = Iout.*2;
rgb = label2rgb(Iout+badCell,jet(2),'m');
figure, imshow(rgb,'InitialMagnification',100)
title(sprintf('CSI (2): Cells Identified (%d cells)',length(ind)),'FontSize',14)
pause(1);

% save results
watershedSelected = ind;
waterImTSeries = waterIm;

save('waterImTSeries.mat','waterImTSeries','watershedSelected','goodCells')

fprintf(1,'CSI/watershedAvgIm: analyzed %d images.\n',size(FNS,2));

end % end watershedAvgIm



function [waterIm, filteredIm] = applyWatershed(rawIm,showImage)
% [waterIm, filteredIm] = applyWatershed(rawIm,showImage)
% Do image conditioning and apply watershed algorithm

if nargin < 2, showImage = false; end

I2 = im2uint16(rawIm);
adjIm = imadjust(I2); % map intensity values to full range
adaIm = adapthisteq(adjIm); % increase image contrast
objWidth = 100;
regMinThreshold = 0.09;
% Convert to double-precision and re-scale.
rawIm = im2double(adaIm);
% Calculate local signal-to-noise ratios.
%  (In a way this corrects for uneven illumination)
snrIm = rawIm ./ imfilter(rawIm, fspecial('average', objWidth),'replicate');
% Enhance contrast by adaptive histogram equalization.
eqIm = adapthisteq(mat2gray(snrIm));
% Filter speckle noise using a 3x3 median filter.
medIm = medfilt2(eqIm,[3 3]);
% Suppress regional minima to mark the basins for the watershed algorithm.
% The cell-cell contacts are supposed to be the ridges.
filteredIm = imhmin(medIm, regMinThreshold);
if showImage
	figure, imshow(filteredIm,'InitialMagnification','fit')
	title('Watershed Input')
end
% Apply watershed algorithm.
waterIm = watershed(filteredIm);
	
end



%#ok<*AGROW>
