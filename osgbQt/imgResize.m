clc; 
clear all;
file_read=dir('D:\ProgramLib\osgbQt\osgbQt\*.jpg');%读取文件夹abc的位置
filenames={file_read.name}';
file_length=length(file_read);
file_length
for i=1:file_length
    iname = ['D:\ProgramLib\osgbQt\osgbQt\' file_read(i).name];
    rename = ['D:\ProgramLib\osgbQt\osgbQt\' file_read(i).name];
    tImg = imread(iname);
   %[width, height] =  size(tImg);
    reImg = imresize(tImg, [512, 512]);
    
    imwrite(reImg, rename);
end
