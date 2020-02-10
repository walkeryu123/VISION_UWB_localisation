function [targetxy] = Plot_TOF(Anchor1_PositionX,Anchor1_PositionY,Anchor2_PositionX,Anchor2_PositionY,Anchor3_PositionX,Anchor3_PositionY,Anchor4_PositionX,Anchor4_PositionY)
    figure(1)
    clf
    AnchorPos = zeros(4,2);
    AnchorPos(1,1) = Anchor1_PositionX;
    AnchorPos(1,2) = Anchor1_PositionY;
    AnchorPos(2,1) = Anchor2_PositionX;
    AnchorPos(2,2) = Anchor2_PositionY;
    AnchorPos(3,1) = Anchor3_PositionX;
    AnchorPos(3,2) = Anchor3_PositionY;
    AnchorPos(4,1) = Anchor4_PositionX;
    AnchorPos(4,2) = Anchor4_PositionY;
    targetxy = circle(5,0,0,1000);
    plot(AnchorPos(:,1),AnchorPos(:,2),'gs','LineWidth',2);
    xlabel('X coordinate of target');
    ylabel('Y coordinate of target');
    title('TW-TOF Localization');
    axis([-10 10 -10 10]);
    drawnow;
    hold on;
end
%%
function [cicxyz] = circle(R,cx,cy,nb_pts)
%%%%%%%%%%%%%%%%%%%
% »­Ô²º¯Êý
%%%%%%%%%%%%%%%%%%%
cicxyz = zeros(nb_pts*2+1,3);
alpha=0:pi/nb_pts:2*pi;%½Ç¶È[0,2*pi]
%R=2;%°ë¾¶
x=R*cos(alpha)+cx;
y=R*sin(alpha)+cy;
cicxyz(:,1) = x';
cicxyz(:,2) = y';
cicxyz(:,3) = 2 ;
end