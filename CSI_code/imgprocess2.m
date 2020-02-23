function imDataOut = imgprocess2(imgFile,medfilter)
%IMDATAOUT = IMGPROCESS2(IMGFILE,MEDFILTER) Process multi-z TIFF stacks
%   Load multi-image .tiff files that represent microscope images at 
%	  different z-heights (a "tiff stack").
%	Apply 2-D median filtering with neighborhood of MEDFILTER. Note that
%	  MEDFILTER = 1 is equivalent to no filtering (image is not changed).
%	Create a single image that is a maximum intensity projection of all
%	  images in the stack.
%	Normalize the pixel intensities in the resulting MIP image to the
%	  max/min of the values in the image.
%
% M. Hopcroft, August 2019
%	hopcroft@reddogresearch.com
%

if nargin < 2, medfilter = 1; end

%% load image data
try
	imData = imread(imgFile);
catch
	imDataOut = []; %#ok<NASGU>
end

%% maximum intensity projection
finfo = imfinfo(imgFile);
num_images = numel(finfo);
imData = zeros([size(imData) num_images]);
for k = 1:num_images
	imData(:,:,k) = medfilt2(imread(imgFile,k),[medfilter medfilter]); % filter image for imgregOfEachCell and register_membrane
end
imMip = max(imData,[],3);

%% normalize pixel intensities
normImBefore = (imMip-min(min(imMip)))/max(max(imMip));
adjIm = imadjust(normImBefore);
medIm = medfilt2(adjIm,[3 3]);
htIm = adapthisteq(medIm);
ftIm = medfilt2(htIm,[3 3]);
imDataOut = ftIm;

end % end function

