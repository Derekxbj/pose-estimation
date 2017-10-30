function position = image_position(I)
% Find the 2D position of points on the image

% I = rgb2gray(I);
    
% Global threshold
W = im2bw(I, graythresh(I)); % White regions
% figure, imshow(W);

radius1 = 2;
S1 = strel('disk', radius1, 0);
W2 = imopen(W,S1);
% figure, imshow(W2);

B = imcomplement(W);
radius2 = 2;
S2 = strel('disk', radius2, 0);
B2 = imopen(B,S2);
% figure, imshow(B2)


[LW, nw] = bwlabel(W2); % Get labeled image LW
blobsWhite = regionprops(LW); % All white blobs
[LB, nb] = bwlabel(B2); % Get labeled image LB
blobsBlack = regionprops(LB); % All black blobs

nccc = 0;
thresh = 1;

for iw = 1:nw
    for ib = 1:nb
        wc = blobsWhite(iw).Centroid;
        bc = blobsBlack(ib).Centroid;
        if norm(wc - bc) < thresh
            % Got a possible CCC; draw a rectangle
             rectangle('Position', blobsWhite(iw).BoundingBox, 'EdgeColor', 'r');
             blobsWhite(iw).BoundingBox;
             nccc = nccc + 1;
             center(nccc,:) = blobsWhite(iw).Centroid;
             
        end
    end

end

position = center;

end


% P = insertText(I,position,'Detected Point', 'FontSize',12);
% figure, imshow(P);