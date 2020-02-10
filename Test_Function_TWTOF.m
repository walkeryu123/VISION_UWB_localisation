function [Estimate_Tag] = Test_Function_TWTOF(LocTag,Anchor1_PositionX,Anchor1_PositionY,Anchor2_PositionX,Anchor2_PositionY,Anchor3_PositionX,Anchor3_PositionY,Anchor4_PositionX,Anchor4_PositionY)

AnchorPos = zeros(4,2);
AnchorPos(1,1) = Anchor1_PositionX;
AnchorPos(1,2) = Anchor1_PositionY;
AnchorPos(2,1) = Anchor2_PositionX;
AnchorPos(2,2) = Anchor2_PositionY;
AnchorPos(3,1) = Anchor3_PositionX;
AnchorPos(3,2) = Anchor3_PositionY;
AnchorPos(4,1) = Anchor4_PositionX;
AnchorPos(4,2) = Anchor4_PositionY;

Estimate_Tag = zeros(1,2);

Estimate_Tag = TW_TOF(LocTag,AnchorPos,1e-10);

if ~exist('Tag.mat','file')
    TagResult = [LocTag Estimate_Tag];
    save('Tag.mat','TagResult','-mat');
else
    tmpData = load('Tag.mat');
    tmp_Result = [tmpData.TagResult];
    ComtmpTag = [LocTag Estimate_Tag];
    TagResult = [tmp_Result;ComtmpTag];
    save('Tag.mat','TagResult','-mat', '-append');
end

% save('EstimateTag.mat','Estimate_Tag');
% save('RealTag.mat','LocTag');
plot(LocTag(1,1),LocTag(1,2),'b-*','LineWidth',1);
plot(Estimate_Tag(1,1),Estimate_Tag(1,2),'r-.x','LineWidth',1);
drawnow;
end
function [Estimate_Tag] = TW_TOF(LocTag,Anchor_Position,NoiseVar)
Estimate_Tag = zeros(1,2);
%%Anchor位置已知
%%
%%初值设定

if NoiseVar==0
    NoiseVar = 1e-10;
end


K_mse = 13;
for kff=1:K_mse         % 方差的复制，最小的方差为2e-10，一次按照2倍的关系增加   
    mse(kff) = 10^((kff-1)/4);
    mse(kff) = mse(kff) * ( 1e-12 ) ;
end
%NoiseVar = 1e-10;           %%噪声方差
Timediv = 0.01;
NumberMeng = 1;
AcN = 4;                    %%Anchor数目
TagData = zeros(NumberMeng,K_mse);
TagNumber = zeros(NumberMeng,2);
m_iRDistance = zeros(AcN,NumberMeng);
m_iMDistance = zeros(AcN,NumberMeng);
m_iMTimeDelay = zeros(AcN,NumberMeng);
realtime = 2;


%%
        c = 300000000;                    %%光速计算
        iround = 1;                %%进行的轮数      

        LocAcN = Anchor_Position;        %%初始化Anchor位置信息矩阵
        %%定义Anchor位置信息
%         AcL = Anchor_AcL;              %%Anchor坐标基准
%         LocAcN(1,1) = 0;
%         LocAcN(1,2) = 0;        %%Anchor1
% 
%         LocAcN(2,1) = AcL;
%         LocAcN(2,2) = 0;        %%Anchor2
% 
%         LocAcN(3,1) = AcL;
%         LocAcN(3,2) = AcL;      %%Anchor3
% 
%         LocAcN(4,1) = 0;
%         LocAcN(4,2) = AcL;      %%Anchor4

        %%定义Anchor之间距离信息矩阵
        AcDis = zeros(4,4);

        AcDis(1,1) = 0;
        AcDis(1,2) = sqrt((Anchor_Position(1,1)-Anchor_Position(2,1))^2+(Anchor_Position(1,2)-Anchor_Position(2,2))^2);
        AcDis(1,3) = sqrt((Anchor_Position(1,1)-Anchor_Position(3,1))^2+(Anchor_Position(1,2)-Anchor_Position(3,2))^2);
        AcDis(1,4) = sqrt((Anchor_Position(1,1)-Anchor_Position(4,1))^2+(Anchor_Position(1,2)-Anchor_Position(4,2))^2);

        AcDis(2,1) = sqrt((Anchor_Position(2,1)-Anchor_Position(1,1))^2+(Anchor_Position(2,2)-Anchor_Position(1,2))^2);
        AcDis(2,2) = 0;
        AcDis(2,3) = sqrt((Anchor_Position(2,1)-Anchor_Position(3,1))^2+(Anchor_Position(2,2)-Anchor_Position(3,2))^2);
        AcDis(2,4) = sqrt((Anchor_Position(2,1)-Anchor_Position(4,1))^2+(Anchor_Position(2,2)-Anchor_Position(4,2))^2);

        AcDis(3,1) = sqrt((Anchor_Position(3,1)-Anchor_Position(1,1))^2+(Anchor_Position(3,2)-Anchor_Position(1,2))^2);
        AcDis(3,2) = sqrt((Anchor_Position(3,1)-Anchor_Position(2,1))^2+(Anchor_Position(3,2)-Anchor_Position(2,2))^2);
        AcDis(3,3) = 0;
        AcDis(3,4) = sqrt((Anchor_Position(3,1)-Anchor_Position(4,1))^2+(Anchor_Position(3,2)-Anchor_Position(4,2))^2);

        AcDis(4,1) = sqrt((Anchor_Position(1,1)-Anchor_Position(1,1))^2+(Anchor_Position(4,2)-Anchor_Position(1,2))^2);
        AcDis(4,2) = sqrt((Anchor_Position(1,1)-Anchor_Position(2,1))^2+(Anchor_Position(4,2)-Anchor_Position(2,2))^2);
        AcDis(4,3) = sqrt((Anchor_Position(1,1)-Anchor_Position(3,1))^2+(Anchor_Position(4,2)-Anchor_Position(3,2))^2);
        AcDis(4,4) = 0;
        
        %%确定标签位置       
         TagNumber = 1;
%         LocTag = zeros(TagNumber,2);    %%初始化真实Tag位置信息矩阵
%         Est_position = zeros(TagNumber,2);
% 
%         for i=1:TagNumber
%             LocTag(i,:) = rand(1,2)*AcL+20;
%         end
%         LocTag(1,1) = 0.7*AcL;  %%标签真实位置信息
%         LocTag(1,2) = 0.4*AcL;  

        %%定义Tag与Anchor之间的距离信息矩阵
        TagAcNDis = zeros(TagNumber,4);
        for i=1:TagNumber
            TagAcNDis(i,1) = sqrt((Anchor_Position(1,1)-LocTag(i,1))^2+(Anchor_Position(1,2)-LocTag(i,2))^2);         %%Tag与Anchor之间真实距离矩阵
            TagAcNDis(i,2) = sqrt((Anchor_Position(2,1)-LocTag(i,1))^2+(Anchor_Position(2,2)-LocTag(i,2))^2);
            TagAcNDis(i,3) = sqrt((Anchor_Position(3,1)-LocTag(i,1))^2+(Anchor_Position(3,2)-LocTag(i,2))^2);
            TagAcNDis(i,4) = sqrt((Anchor_Position(4,1)-LocTag(i,1))^2+(Anchor_Position(4,2)-LocTag(i,2))^2);
        end
        
%         for i=1:TagNumber
%             TagAcNDis(i,1) = sqrt((LocTag(i,1))^2+(LocTag(i,2))^2);         %%Tag与Anchor之间真实距离矩阵
%             TagAcNDis(i,2) = sqrt((AcL-LocTag(i,1))^2+(LocTag(i,2))^2);
%             TagAcNDis(i,3) = sqrt((AcL-LocTag(i,1))^2+(AcL-LocTag(i,2))^2);
%             TagAcNDis(i,4) = sqrt((LocTag(i,1))^2+(AcL-LocTag(i,2))^2);
%         end
                
        
 %%
 for m_iTagNum = 1:TagNumber
     
        m_tMTOD = zeros(AcN,NumberMeng);    %%初始化Tag发出信号测量时间信息矩阵
        m_tTTOD = zeros(AcN,NumberMeng);    %%初始化Tag发出信号真实时间信息矩阵
        m_tMTOA = zeros(AcN,NumberMeng);    %%初始化Tag接收信号测量时间信息矩阵
        m_tTTOA = zeros(AcN,NumberMeng);    %%初始化Tag接收信号真实时间信息矩阵

        m_aMTOD = zeros(AcN,NumberMeng);    %%初始化Anchor发出信号测量时间信息矩阵
        m_aTTOD = zeros(AcN,NumberMeng);    %%初始化Anchor发出信号真实时间信息矩阵
        m_aMTOA = zeros(AcN,NumberMeng);    %%初始化Anchor接收信号测量时间信息矩阵
        m_aTTOA = zeros(AcN,NumberMeng);    %%初始化Anchor接收信号真实时间信息矩阵
        
        m_aMTWTOF = zeros(AcN,NumberMeng);    %%初始化TWTOF测量信息矩阵
        m_aTTWTOF = zeros(AcN,NumberMeng);    %%初始化TWTOF真实信息矩阵
        
%%        m_iFRDistance = zeros(NumberMeng,K_mse);
        m_iFMDistanceA1 = zeros(NumberMeng,K_mse);
        m_iFMDistanceA2 = zeros(NumberMeng,K_mse);
     
    for m_iNoise=1:1
        
        NoiseT1 = random('Normal',0,NoiseVar,AcN+1,NumberMeng);     %%噪声信息矩阵
        NoiseT2 = random('Normal',0,NoiseVar,AcN+1,NumberMeng);     %%噪声信息矩阵
        NoiseA1 = random('Normal',0,NoiseVar,AcN+1,NumberMeng);     %%噪声信息矩阵
        NoiseA2 = random('Normal',0,NoiseVar,AcN+1,NumberMeng);     %%噪声信息矩阵
        
        for m_jNumberMeng=1:NumberMeng
            %%


            %%计算原始数据

            for i=1:AcN
                m_tTTOD(i,m_jNumberMeng) = realtime + i*Timediv;     %%产生真实Tag发射信号时间，每个Anchor间隔0.001s          
                m_tMTOD(i,m_jNumberMeng) = m_tTTOD(i,m_jNumberMeng) + NoiseT1(1,m_jNumberMeng);  %%产生测量Tag发射信号时间
                m_aTTOA(i,m_jNumberMeng) = TagAcNDis(m_iTagNum,i)/c + m_tTTOD(i,m_jNumberMeng);  %%产生真实Anchor接收信号时间
                m_aMTOA(i,m_jNumberMeng) = m_aTTOA(i,m_jNumberMeng) + NoiseA1(i+1,m_jNumberMeng);  %%产生测量Anchor接收信号时间
                m_aTTOD(i,m_jNumberMeng) = m_aTTOA(i,m_jNumberMeng) + Timediv;     %%产生真实Anchor发射信号时间，间隔0.001s
                m_aMTOD(i,m_jNumberMeng) = m_aTTOD(i,m_jNumberMeng) + NoiseA2(i+1,m_jNumberMeng);  %%产生测量Anchor发射信号时间
                m_tTTOA(i,m_jNumberMeng) = TagAcNDis(m_iTagNum,i)/c + m_aTTOD(i,m_jNumberMeng);  %%产生真实Anchor发射信号时间
                m_tMTOA(i,m_jNumberMeng) = m_tTTOA(i,m_jNumberMeng) + NoiseT2(1,m_jNumberMeng);  %%产生测量Anchor发射信号时间          
            end
            for i=1:AcN
                m_iMTimeDelay(i,m_jNumberMeng) = (((m_tMTOA(i,m_jNumberMeng)-m_tMTOD(i,m_jNumberMeng))-(m_aMTOD(i,m_jNumberMeng)-m_aMTOA(i,m_jNumberMeng)))/2);
                m_iRDistance(i,m_jNumberMeng) = (((m_tTTOA(i,m_jNumberMeng)-m_tTTOD(i,m_jNumberMeng))-(m_aTTOD(i,m_jNumberMeng)-m_aTTOA(i,m_jNumberMeng)))/2)*c;
                m_iMDistance(i,m_jNumberMeng) = (((m_tMTOA(i,m_jNumberMeng)-m_tMTOD(i,m_jNumberMeng))-(m_aMTOD(i,m_jNumberMeng)-m_aMTOA(i,m_jNumberMeng)))/2)*c;
            end
    %        m_iFRDistance(m_jNumberMeng,m_iNoise) = abs(m_iRDistance(1,m_jNumberMeng)-TagAcNDis(1,1));
            m_iFMDistanceA1(m_jNumberMeng,m_iNoise) = abs(m_iMDistance(1,m_jNumberMeng)-TagAcNDis(m_iTagNum,1));
            m_iFMDistanceA2(m_jNumberMeng,m_iNoise) = abs(m_iMDistance(2,m_jNumberMeng)-TagAcNDis(m_iTagNum,2));
        end
    
%%
            %%%%%%%通过最大似然估计计算标签位置信息
            %%%%%%% Z = f(x0) + N,f(x0)为Anchor与Tag之间距离的距离差（通过测量TOA信息计算得出）
            c_us = 300;
        for m_jNumberMeng=1:NumberMeng
                    C = zeros(4,5);                     %%初始化系数矩阵
                    for i=1:4
                        C(i,1) = 1;
                        C(i,i+1) = -1;
                    end
                    C1 = zeros(4,5);                     %%初始化系数矩阵
                    for i=1:4
                        C1(i,1) = -1;
                        C1(i,i+1) = 1;
                    end
                    Qu = eye(5,5);                    %%初始化噪声方差阵
                    Qu = Qu*NoiseVar*NoiseVar;
                    Qu1 = eye(5,5);                    %%初始化噪声方差阵
                    Qu1 = Qu1*NoiseVar*NoiseVar;               


                    Qw = zeros(4,4);                    %%W = C*N;Qw表示W的方差阵
                    Qw = C*Qu*C'+C1*Qu1*C1';
                    Qw = diag(diag(Qw));
                    Qw1 = inv(Qw);
    %                 Qw1 = Qw1 /1000000000000;


                    TagPos = rand(1,2);             %%赋给Tag迭代的初值
                    %%迭代求解估计Tag位置信息
                    k = 0;                              %%迭代次数
                    f = zeros(AcN,1);
                    while k<1000
                        %%给定f(x0)
                        for i=1:4
                            f(i,1) = norm(LocAcN(i,:)-TagPos,2);
                        end
                        f = f/c;
                        %%求解G矩阵(雅克比矩阵)
                        %%按照微妙计算
                        for i=1:4
                            G(i,:) = (LocAcN(i,:)-TagPos)/norm(LocAcN(i,:)-TagPos,2);
                            %G(i,:) = G(i,:)-(LocAcN(AnchorNum(1,1),:)-TagPos)/norm(LocAcN(AnchorNum(1,1),:)-TagPos,2);
                        end
                        G = -G/c;


                    %     Qw1 = Qw1/1000000000000;
                        Qa = inv(G'*Qw1*G);


                        Qb = G'*Qw1;
                        Qd = m_iMTimeDelay(:,m_jNumberMeng)-f(:,1);
                        Delta = Qa*Qb*Qd;

                        TagPos1 = TagPos;
                        TagPos = TagPos+Delta';
                        TagPos2 = TagPos-TagPos1;
                        if abs(TagPos2(1,1))<0.0000001
                            if abs(TagPos2(1,2))<0.0000001
    %                             for i=1:4
    %                                 G1(i,:) = (LocAcN(i,:)-TagPos)/norm(LocAcN(i,:)-TagPos,2);
    %                             end
    %                             G1 = -G1/c_us;
    %                             CRLB_tag =  inv(G1'* Qw1 * G1) ;
    %                             CRLB_F(m_jNumberMeng,m_iNoise) = CRLB_tag(1,1)+CRLB_tag(2,2);
                                break;
                            end
                        end
                        k = k + 1;
                    end

                    if isnan(TagPos(1,:))
                        if m_meng>1
                            %TagNumber(m_jNumberMeng,:) = TagNumber(m_jNumberMeng-1,:);
                        end
                    else
                        Estimate_Tag(m_iTagNum,:) = TagPos(1,:);
                        %TagNumber(m_jNumberMeng,:) = TagPos(1,:)-[70 40];
                    end


        end
    
    end
 end

end