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
% %����uint8����ͼ��
% I_uint8=imread(path);
% %��uint8����ͼ��תΪdouble�ͣ����һὫ
% %�Ҷ�[0,255]�Զ���һ��[0,1]
% I = im2double(I_uint8);
% imwrite(I,'out.png');%����ͼ��
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

