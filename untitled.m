%pyversion D:\Program Files\python3.7\python.exe
clc
clear
% 
%  import py.testyby.*
% mod = py.importlib.import_module('testyby'); 
% % 
% % py.importlib.reload(mod);
% 
% % aa = py.sys.path;
% P = py.sys.path;
% if count(P,'modpath') == 0
%     insert(P,int32(0),'modpath');
% end
% img = py.cv2.imread('E:\test.jpg');
% py.cv2.imshow('img',img);
% 
% if count(py.sys.path,'') == 0 
%     insert(py.sys.path,int32(0),''); 
% end
% %r=py.sum(2,2)
% py.cv2.imread();
% % res = sum(5, 7)
% 
% path=sprintf('E:/test.jpg');
% %读入uint8类型图像
% I_uint8=imread(path);
% %将uint8类型图像转为double型，并且会将
% %灰度[0,255]自动归一化[0,1]
% I = im2double(I_uint8);
% imwrite(I,'out.png');%保存图像
 %mod = py.('mymod.py'); 
% 
% py.importlib.reload(mod);
% py.importlib.reload();
P = py.sys.path;
if count(py.sys.path,'') == 0
    insert(py.sys.path,int32(0),'');
end



% N = py.cv2.aruco_DetectorParameters.list({'Jones','Johnson','James'});
a = 1;
b = 3;
c = py.mymod.sum(a,b);

