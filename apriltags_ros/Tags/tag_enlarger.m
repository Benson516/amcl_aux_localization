
clc

%%

tagFamily = 'tag36h11'
folder = [tagFamily, '/', tagFamily, '/'];
folder_save = [folder, 'enlarged/']
[~,~,~] = mkdir(folder_save);
%
scale = 50
for tag = 0:586
    % fileName = 'tag36_11_00000.png'
    fprintf('-------------\n# of tag: %d', tag);
    fileName = sprintf('tag36_11_%05d.png', tag)
    filePath = [folder, fileName];
    
    % Load image
    I = imread(filePath);
    
    % Resize
    IL = imresize(I, scale,'nearest');
    % imshow(IL)
    
    % Save image
    fileName_save = ['L_', fileName]
    imwrite(IL, [folder_save, fileName_save])
end