% run_CSI
%  RUN_CSI: Cell Strain from Images
%
% This MATLAB script calculates strain from a series of microscope images. 
%  The script was designed to process images of a cell monolayer tagged for
%  E-cadherin (i.e. images of cell membranes). Each image should be saved
%  as a TIFF file. Each TIFF file may be a z-stack representing the image
%  at different focal planes. The images should be numbered sequentially.
%  When sorted, the first image (lowest number in the naming sequence) is
%  assumed to be a "zero-strain" image, and subsequent images are compared
%  to it.
%
% To use these scripts, first make sure that the files are on the MATLAB
%  path. In MATLAB, go to the directory containing these scripts and enter
%  the command "addpath(pwd)". Now change to the directory with your image
%  files in MATLAB and start the script by typing "run_CSI" at the command
%  line.
%
% Note that there are some user-modifiable constants at the beginning of
%  this script file. In particular, you should enter an estimated value
%  for cell size in pixels as the variable "cellNperLength" (see below).
%
% The run time for this analysis may be more than an hour. Because the
%  script analyses the strain of individual cells in the image, the runtime
%  scales linearly with the number of cells in the image.
%
% Before using this script on a new set of images, you should create a
%  series of test images based on one of your microscope images and use
%  these test images to validate this script for the new images. Use the
%  script "make_test_images.m" to create the test images. Four test images
%  is sufficient. Run this script on the test images and verify that the
%  strain results are as expected from the test images.
%
% Required files:
%	run_CSI.m
%	imgprocess2.m
%	register_membrane.m
%	watershedAvgIm.m
%	imregOfEachCell.m
%	sort_nat.m
%

% Authors:
%	Joo-Yong Sim
%	Matthew A. Hopcroft, hopcroft@reddogresearch.com
%	
%  February 2020
% 

%% User-modifiable constants
% The estimated number of cells along the x-dimension of the initial image.
%  This value is used to filter incorrectly-identified cells on the basis
%  of size- objects that are much smaller or larger than the expected cell
%  size will be ignored.
% NOTE: This assumes an approximately square image
cellNperLength = 30;
% Pressure Steps [kPa]
%  The final strain results will be plotted against these values (i.e.,
%   these are the x-axis values for the results)
%  Leave empty for default x-axis (step number)
pr=[];
% pr = [0 15 30 45 60 75]; % kPa


%% Start Analysis
tstart = now;
fprintf(1,'\nrun_CSI: Start at %s\n\n',datestr(tstart));
rundate = datestr(tstart,'yyyymmddHHMM');


%% Step 0: get list of TIFF files to work with
fprintf(1,'CSI: working directory:\n');
disp(pwd)
FN = dir('*.tiff');
FNL = {FN.name};
FNS = sort_nat(FNL); % sort_nat: Natural order sort of cell array of strings.
disp('Using .tiff files found in working directory:')
disp(FNS')


%% Step 1: register each image to the zero-strain image
register_membrane(FNS); % save('tform.mat','tformMembrane')


%% Step 2: find cell boundaries
watershedAvgIm(FNS,cellNperLength); % save('waterImTSeries.mat','waterImTSeries','watershedSelected') save('avgImg.mat','avgImg','filteredIm','avgProp')


%% Step 3: find deformation of each cell
imregOfEachCell(FNS,cellNperLength); % save('tformLocals.mat','TFORM') save('cellProperties.mat','cellArea','cellPerm','cellEccn')


%% Step 4: determine average cell strains

% load results of previous steps
load('avgImg.mat')
load('tform.mat')
load('tformLocals.mat')
load('waterImTSeries.mat')


avgExx = [0];
avgExy = [0];
stdExy = [0];
stdExx = [0];
avgEyy = [0];
stdEyy = [0];
Exx = {0};
Eyy = {0};
Exy = {0};

% loop over results from all files
%  skip the first file because it is the baseline image (0% strain)
for i = 2:size(FNS,2)
	%temp = tformMembrane{i};
	temp = tformCell{i};
	RD = [];
	RD1 = RD;
	RD2 = RD;
	RD3 = RD;

	ind = watershedSelected;
	for j = 1:length(ind)
		rd = TFORM{i,j};
% 		% filter outlier values that represent bad cells
% 		if (abs(1-rd(1,1))>=0.2)||(abs(1-rd(2,2))>=0.2)||(abs(rd(1,2))>=50)||(abs(rd(2,1))>=50)
% 		else

			rd2 = rd*temp.T;
			%RD = [RD;rd2];
			RD1 = [RD1;rd2(1,1)];
			RD2 = [RD2;rd2(2,2)];
			RD3 = [RD3;rd2(1,2)];

% 		end
	end
	
	% these are the average strains for all cells in the image
	avgEyy(i) = mean(1./RD2-1);
	stdEyy(i) = std(1./RD2-1);
	avgExx(i) =  mean(1./RD1-1);
	stdExx(i) =  std(1./RD1-1);
	avgExy(i) =  mean(RD3);
	stdExy(i) =  std(RD3);

	% these are the individual cells strains in each image 
	Exx{i} = 1./RD1-1;
	Eyy{i} = 1./RD2-1;
	Exy{i} = RD3;

end

% save results to workspace
save(['CSI_result_' rundate '.mat'],'avgExx','avgEyy', 'avgExy', 'stdExx',...
    'stdEyy', 'stdExy','Exx', 'Eyy', 'Exy', 'tstart', 'rundate', 'FNS','pr');
%save('all_var.mat')


%% Plot the Strain Results
if isempty(pr)
	pr = 0:1:size(FNS,2)-1;
end

% plot average cell strain for each pressure (image)
figure

pxx = plot(pr,avgExx,'O-','Color',[0.85 0.32 0.098]);
hold on
pyy = plot(pr,avgEyy,'O-','Color',[0.46 0.67 0.18]);
pxy = plot(pr,avgExy,'O-','Color',[0 0.44 0.74]);

% plot the strain estimated from the whole image, as a sanity check on the
%  cell-based strain values
try
	load('image_reg_strains.mat','imexx','imeyy','imexy','imeyx','pri');
	ixx = plot(pr,imexx,'s-r','MarkerSize',12);
	iyy = plot(pr,imeyy,'s-g','MarkerSize',12);
	ixy = plot(pr,imexy,'s-b','MarkerSize',12);
catch
	fprintf('CSI: Unable to load strain results from image registration\n');
end


% use std for errorbars
errorbar(pr,avgExx,stdExx,'Color',[0.854 0.326 0.098])
errorbar(pr,avgEyy,stdEyy,'Color',[0.46 0.67 0.18])
errorbar(pr,avgExy,stdExy,'Color',[0 0.44 0.74])

% set the axis tick marks
ax = gca;
ax.XAxis.TickValues=pr;

% set title, axis etc
title('CSI: Cell Strains','Interpreter','None')
ylabel('Strain [\deltas/s]')
xlabel('Pressure Step [#]')

% show strain estimated from whole image for sanity check
try
	legend([pxx pyy pxy ixx],{'e_x_x (mean)','e_y_y (mean)','e_x_y (mean)','e_x_x (whole image)'},'Location','northwest')
catch
	legend([pxx pyy pxy],{'e_x_x (mean)','e_y_y (mean)','e_x_y (mean)'},'Location','northwest')
end
grid on
savefig(['CSI_result_' rundate]);

%% Finish
tend = now;
fprintf(1,'\nrun_CSI: Elapsed time: %.1f sec\n\n',(tend-tstart)*86400);

%#ok<*AGROW>
%#ok<*SAGROW>
%#ok<*NBRAK>
