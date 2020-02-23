function imregOfEachCell(FNS,cellSize)
% IMREGOFEACHCELL(FNS) estimates strain in each cell by comparing it to the
%  the same cell in the zero-strain image.
% (CSI Step #3)
%  

debug = 0; % set to one to display QA/debug images of the analysis

fprintf(1,'\nCSI/imregOfEachCell\n');
warning('off','images:regmex:registrationFailedException');

% load results from previous steps
load('waterImTSeries.mat','watershedSelected','waterImTSeries')
load('tform.mat','tformCell')
load('avgImg.mat','filteredIm','avgProp','greenIm','bwIm');
ind = watershedSelected;
waterIm = waterImTSeries;

% load the fixed ("zero-strain") image
fixedAd = imgprocess2(FNS{1},2);
%fixedAd = filteredIm;

%% estimate strain for each cell in each image
%
% figures for QA/debug (optional)
if debug
	imfig = figure;
	cellfig = figure;
end

% imregister settings
[optimizer, metric] = imregconfig('multimodal');
optimizer.InitialRadius = 0.0005;
optimizer.Epsilon = 1.5e-8;
optimizer.GrowthFactor = 1.005;
optimizer.MaximumIterations = 500;

TFORM = {};

cellArea = zeros(length(ind),size(FNS,2));
cellPerm = zeros(length(ind),size(FNS,2));
cellEccn = zeros(length(ind),size(FNS,2));
% get cell properties for "baseline average" image
cellProp = regionprops(waterIm,'Eccentricity','Perimeter','Area');
cellProp = cellProp(ind);
cellArea(:,1) = cat(1,cellProp.Area);
cellPerm(:,1) = cat(1,cellProp.Perimeter);
cellEccn(:,1) = cat(1,cellProp.Eccentricity);

fprintf(1,'CSI/imregOfEachCell: register cells in each image file\n');
% loop over all images
for i = 2:size(FNS,2)

	tstart = now;
	
	% read the next image
	fprintf(1,' Image: %d / %d start at %s: %s\n',i,size(FNS,2),datestr(tstart),FNS{i});	
	movingAd = imgprocess2(FNS{i},2);

	% transform the image using the transform determined in step 1
	movingRegistered = imwarp(movingAd,tformCell{i},'OutputView',imref2d(size(fixedAd)));
	
	if debug
		figure(imfig);
		imshow(movingRegistered)
		rectangle('Position',avgProp.BoundingBox,'EdgeColor','blue');
		%imshowpair(fixedAd, movingRegistered,'diff','Scaling','joint');
		title(FNS{i},'Interpreter','none')
	end
	
	% crop to the area common to the average image
	movingRegistered = imcrop(movingRegistered,avgProp.BoundingBox);

	if debug, drawnow; disp('Press any key to continue.'); pause; end

	%% find deformation of each cell
	fprintf(1,' local imregistration for %d cells\n',length(ind));
	wb = waitbar(0,sprintf('%s (%d cells)',FNS{i},length(ind)),'Name',sprintf('Register Cells (%d / %d)',i,size(FNS,2)));
	wb.Children.Title.Interpreter = 'none';
	% loop over cells
	for j = 1:length(ind)
		
		% load indexed image ROI
		selectedReg = waterIm == ind(j);
		
		% dilate the mask to ensure that we have enough pixels to match
		%  the region reliably during the transform
		se = strel('diamond',round(cellSize/2));
		diIm = imdilate(selectedReg,se);

		% find rectangle bounding box of this cell region
		s = regionprops(diIm,'BoundingBox');
		cFixedIm = imcrop(filteredIm,s.BoundingBox);
		cMovingIm = imcrop(movingRegistered,s.BoundingBox);
		% for debug visualization
		%waterCell = imcrop(greenIm,s.BoundingBox);
		%bwCell = imcrop(bwIm,s.BoundingBox);
		if debug, selectedCell = imcrop(selectedReg,s.BoundingBox); end
		
		% register the image of this cell region
		tform = imregtform(cMovingIm,cFixedIm,'affine',optimizer,metric);
		TFORM{i,j} = tform.T; %#ok<AGROW>
		
		% estimate individual cell change by applying transform to this
		%  cell boundary
		invTlocal = invert(tform);
		invTglobal = invert(tformCell{i});
		invT = affine2d(invTlocal.T * invTglobal.T);
		selectedCellStrain = imwarp(selectedReg,invT,'nearest');
		scp = regionprops(selectedCellStrain,'Eccentricity','Perimeter','Area');
		cellArea(j,i) = scp.Area;
		cellPerm(j,i) = scp.Perimeter;
		cellEccn(j,i) = scp.Eccentricity;
		
		% DEBUG - show individual cell images
		if debug
			sp = regionprops(selectedReg,'BoundingBox');
			figure(imfig);
			imshowpair(filteredIm, movingRegistered,'Scaling','joint');
			hold on
			image('CData',greenIm,'AlphaData',bwIm); % the watershed result (cell regions)
			visboundaries(diIm,'Color','red'); % the cell boundary (dilated)
			rectangle('Position',s.BoundingBox,'EdgeColor','blue'); % the bounding box for this region
			rectangle('Position',sp.BoundingBox,'EdgeColor','yellow'); % the bounding box for this region
			title('Individual Cell ROI')
			hold off

			figure(cellfig)
			subplot(1,2,1)
			hold off
			imshow(cFixedIm)
			hold on
			%image('CData',waterCell,'AlphaData',bwCell); % show the waterfall boundaries
			visboundaries(selectedCell,'Color','red'); % show selected cell
			title('cFixedIm')
			subplot(1,2,2)
			imshow(cMovingIm)
			title('cMovingIm')
			fprintf('(Displaying cell images. Press any key to continue.)\n')
			pause
		end
		
		% update waitbar
		waitbar(j/length(ind),wb)
		drawnow;

	end
	

	close(wb)
	tend = now;
	fprintf(1,'CSI/imregOfEachCell: Loop elapsed time: %.0f seconds. End at %s\n',(tend-tstart)*86400,datestr(tend));
	drawnow;
	
end
fprintf(1,'imregOfEachCell: analyzed %d images.\n',length(tformCell)-1);

save('tformLocals.mat','TFORM')
save('cellProperties.mat','cellArea','cellPerm','cellEccn');

%#ok<*UNRCH>

