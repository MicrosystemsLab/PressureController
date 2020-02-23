function register_membrane(FNS)
% REGISTER_MEMBRANE(FNS) registers the deformation of a series of
%  microscope images relative to the first image in the series.
% (CSI Step #1)
%

fprintf(1,'\nCSI/register_membrane\n');

% load the fixed ("zero-strain") image
fprintf(1,'CSI/register_membrane: baseline (zero-strain) image is "%s"\n',FNS{1});
fixedAd = imgprocess2(FNS{1},2);

figure, imshow(fixedAd,'InitialMagnification',100);
title(['CSI (1) zero strain: ' FNS{1}],'Interpreter','none')
drawnow;

%% initialize transform parameters
[optimizer, metric] = imregconfig('multimodal');
optimizer.InitialRadius = 0.0005;
optimizer.Epsilon = 1.5e-8;
optimizer.GrowthFactor = 1.006;
optimizer.MaximumIterations = 600;
% establish the baseline transform (i.e. no movement)
%tformCell{1} = imregtform(fixedAd,fixedAd,'affine',optimizer,metric);
tformCell{1} = affine2d([1 0 0;0 1 0;0 0 1]);


%% process all images
% Determine the deformation of each image relative to the baseline
%  ("zero-strain") image. This provides a good estimate of average
%  strain in the monolayer, but does not estimate strain in each cell.

% save strains estimated from image
imexx = zeros(1,size(FNS,2));
imeyy = zeros(1,size(FNS,2));
imexy = zeros(1,size(FNS,2));
imeyx = zeros(1,size(FNS,2));

% set conditions for image transform
[optimizer, metric] = imregconfig('multimodal');
optimizer.InitialRadius = 0.0005;
optimizer.Epsilon = 1.5e-8;
optimizer.GrowthFactor = 1.006;
optimizer.MaximumIterations = 600;

% loop over all images
fprintf(1,'CSI/register_membrane: register each image to baseline\n');
for i = 2:size(FNS,2)
	tloop = now;
	% load an image
	fprintf(1,'Image: %d / %d: %s ',i,size(FNS,2),FNS{i});
	% imgprocess2: apply image filtering and create single image (maximum
	%  intensity projection) from all images in z-stack
	movingAd = imgprocess2(FNS{i},2);
	
	% use imregtform to estimate the distortion of the image relative to
	%  the baseline zero-strain image
	tformInit = tformCell{i-1};
	movingRegInit = imwarp(movingAd,tformInit,'OutputView',imref2d(size(fixedAd)));

	tformInter = imregtform(movingRegInit,fixedAd,'affine',optimizer,metric);
	tformi = tformInit;
	tformi.T = tformInter.T*tformInit.T;
	tformCell{i} = tformi; %#ok<AGROW>

	% quality check
	regqual = tformCell{i-1}.T/(tformCell{i}.T);
	idm = [1 0 0;0 1 0;0 0 1];
	regqual0 = [ones(2,3)*1e3;ones(1,3)].*abs(idm - regqual);

	
	% save strain estimates
	imexx(i) = tformi.T(1,1);
	imeyy(i) = tformi.T(2,2);
	imexy(i) = tformi.T(1,2);
	imeyx(i) = tformi.T(2,1);
	
 	% optional: show comparison of the fixed and moving image
 	imshowpair(fixedAd, imwarp(movingAd,tformCell{i},'OutputView',imref2d(size(fixedAd))),'Scaling','joint');
	title(sprintf('CSI (1): Image Pair %d',i),'Interpreter','none')
% 	pause
	
	drawnow;
    
	fprintf(1,'(%.0f sec / score %.1f)\n',(now-tloop)*86400,sum(sum(regqual0)));
	%disp(regqual0)
end

drawnow;
fprintf(1,'CSI/register_membrane: registered %d images.\n',length(tformCell)-1);
pause(2); close;

% estimate strains from image registration
imexx = 1./imexx-1; imexx(1) = 0;
imeyy = 1./imeyy-1; imeyy(1) = 0;
max_xx = max(imexx); max_yy = max(imeyy);
pri = 1:1:size(FNS,2);
figure
plot(pri,imexx,'o-',pri,imeyy,'s-',pri,imexy,'^-',pri,imeyx,'*-');
title('CSI (1): Strains from Whole Image')
xlabel('Image Number')
ylabel('Strain [\deltas/s]')
legend('e_x_x','e_y_y','e_x_y','e_y_x','Location','northwest')
grid on
drawnow;

% save results
save('tform.mat','tformCell','max_xx','max_yy')
save('image_reg_strains.mat','imexx','imeyy','imexy','imeyx','pri')

