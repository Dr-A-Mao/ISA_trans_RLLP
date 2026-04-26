% 用以计算某个路径的一系列属性的函数
function [path_temp,dist_temp,time_temp,cost_temp] = get_path_temp(beginlat,beginlng,endlat,endlng,MaxAngle,vel)
if nargin == 0
    beginlat = 44.8737;
    beginlng = 128.0712;
    endlat = 44.7962;
    endlng = 128.1365;
    MaxAngle = 10;
    vel = 30;
end

if nargin == 4
    MaxAngle = 10;
    vel = 30;
end
% 读取重要数据
try 
    demyuanshi = evalin('base','demyuanshi');
    map_lat = evalin('base','map_lat');
    map_lng = evalin('base','map_lng');
    map_scale = evalin('base','map_scale');
catch
    load('data.mat');
    demyuanshi = evalin('base','demyuanshi');
    map_lat = evalin('base','map_lat');
    map_lng = evalin('base','map_lng');
    map_scale = evalin('base','map_scale');
end
lon_scale = 110e3;
lat_scale = 113e3;

%% 模式选择
TFTA_modle=3;%1是航程最优，2是时间最优，3是地形回避最优
juli_model=1;%1是欧几里得，2是曼哈顿，3是对角线，4是切比雪夫距离
chazhi_model=3;%1是双线性插值，2是双立方，3是二维三次卷积
chafen_model=3;%1是简单差分，2是二阶差分，3是三阶反距离平方权，4是三阶反距离权，5是三阶不带权，6是边框差分

%% 飞机限制
feijixianzhi.binangle=30;%二值图安全坡度
feijixianzhi.Maxpashengjiao=10;%最大爬升角度
feijixianzhi.MinTFlidigaodu=14;%TF离地高度范围
feijixianzhi.MaxTFlidigaodu=24;
feijixianzhi.buchang=180;%步长
feijixianzhi.MaxAngle=MaxAngle;%最大转弯角度
%设定好的
switch TFTA_modle
    case 1
        feijixianzhi.minpoduangle=10;%灰度坡度角范围
        feijixianzhi.maxpoduangle=45;
        feijixianzhi.poduquanzhong=0.5;%坡度权重
        feijixianzhi.juliquanzhong=10;%距离权重
    case 2
        feijixianzhi.minpoduangle=0;%灰度坡度角范围
        feijixianzhi.maxpoduangle=45;
        feijixianzhi.poduquanzhong=0.5;%坡度权重
        feijixianzhi.juliquanzhong=10;%距离权重
    case 3
        feijixianzhi.minpoduangle=0;%灰度坡度角范围
        feijixianzhi.maxpoduangle=30;
        feijixianzhi.juliquanzhongG=0.4;%距离权重
        feijixianzhi.juliquanzhongH=0.5;%距离权重
        feijixianzhi.poduquanzhong=0.1;%坡度权重
end

%%
%起止点选择
[Bounding,anglegray,mapsize,sizeyuanshi]=TFTA_makebin(demyuanshi,map_lng,map_lat,map_scale,feijixianzhi,chazhi_model,chafen_model);
[realpath,realpath_size,~]=TFTA_AstarFunction_new(juli_model,anglegray,Bounding,mapsize,beginlat,beginlng,endlat,endlng,feijixianzhi);
%高度后处理
realpath(1:realpath_size,3)=gaodu(demyuanshi,realpath(1:realpath_size,1:2),Bounding,sizeyuanshi,feijixianzhi); 
%% 关键属性计算
path_temp = realpath(1:realpath_size,:);
path_diff_lon = diff(path_temp(:,1)) * lon_scale;
path_diff_lat = diff(path_temp(:,2)) * lat_scale;
path_diff_alt = diff(path_temp(:,3)) * 1;
% 计算中间结点
dist_temp = norm([path_diff_lon,path_diff_lat,path_diff_alt]);
time_temp = dist_temp/vel;
cost_temp = dist_temp; % 默认cost_star为dist_star这里可能要改!!!!!!!!!!!
end

%% 地图导入函数
function [demyuanshi,map_lng,map_lat,map_scale]=dem_lode()
[mapname,input_path,~]=uigetfile({'*.tif';'*.hgt';'*.mat'});
dem_lodemode=mapname(end-2:end);
switch dem_lodemode
    case 'mat'
        load([input_path,mapname]);
%         demyuanshi=A;
%         map_lng=108;
%         map_lat=34;
        
%         demyuanshi=Astar.demyuanshi;
        map_lng=117.35702587;
        map_lat=24.84667199;
        
        map_scale=1/3;
    case 'hgt'
        pathmapname=[input_path,mapname];
        openHgt=fopen(pathmapname);
        demyuanshi=fread(openHgt,[1201,1201],'short','b'); %读取高程地图数据
        demyuanshi=demyuanshi(:,end:-1:1)';%地图数据是高纬度开始的，转换成低纬度开始
        demyuanshi=demyuanshi(1:1200,1:1200);
        map_lat=str2double(pathmapname(end-9:end-8))+1;                    %纬度
        map_lng=str2double(pathmapname(end-6:end-4));                    %经度
        map_scale=3;
    case 'tif'
        pathmapname=[input_path,mapname];
        info = geotiffinfo([pathmapname]);
        map_lng=info.BoundingBox(1,1);
        map_lat=info.BoundingBox(2,2);
        
%         [map_lng,map_lat]=GaussToGeo(info.BoundingBox(2,2),info.BoundingBox(1,1),120);
        
        demyuanshi= imread([pathmapname]);
        demyuanshi=demyuanshi(end:-1:1,:);
%         demyuanshi=demyuanshi(1:3600,1:3600);
        map_scale=0.323601809472634;
end
% f0=demyuanshi==-9999;%-9999是无效值
% demyuanshi(f0)=0;
f0=demyuanshi==-32767;%-32768是无效值
demyuanshi(f0)=0;
end

%% 规划后计算函数
%规划后的计算
%输入：航点、航点数、飞机限制、地图边界、原始地形、原始地图大小
%输出：航点、全航迹点、全航迹点数、贴地计算结果
%     realpath
%     【经度、纬度、飞机高度、两点之间步长（m）】
function [realpath_all,realpath_all_num,jisuanTF]=culc_after(realpath,realpath_size,feijixianzhi,Bounding,demyuanshi,sizeyuanshi)    
    [huchang_dis,az]=distance(realpath(1:realpath_size,2),realpath(1:realpath_size,1),realpath([1,1:realpath_size-1],2),realpath([1,1:realpath_size-1],1));
    az(3:end,2)=az(3:end,1)-az(2:end-1,1);
    realpath(2:realpath_size,4)=huchang_dis(2:end)/180*pi*6371004;
    realpath(3:realpath_size,5)=az(3:realpath_size,1);
    gaoducha=realpath(2:realpath_size,3)-realpath(1:realpath_size-1,3);
    realpath(2:realpath_size,6)=rad2deg(atan2(gaoducha,realpath(2:realpath_size,4)));
    
    jisuanTF.juli_min=min(realpath(2:realpath_size,4));
    jisuanTF.juli_max=max(realpath(2:realpath_size,4));
    jisuanTF.juli_mean=mean(realpath(2:realpath_size,4));
    jisuanTF.az_max=max(abs(az(3:end,2)));
    jisuanTF.az_mean=mean(abs(az(3:end,2)));
    jisuanTF.podu_max=max(abs(realpath(2:realpath_size,6)));
    jisuanTF.podu_mean=mean(abs(realpath(2:realpath_size,6)));
   
    %补充中间点
    chazhi_num=10;%两个点中间插入10个点
    buchang=(realpath(2:realpath_size,1:3)-realpath(1:realpath_size-1,1:3))/(chazhi_num+1);
    realpath_all=zeros((realpath_size-1)*(chazhi_num+1)+1,8);
    realpath_alltemp=zeros(chazhi_num+1,3);
    for i=1:realpath_size-1
        for j=0:chazhi_num
            realpath_alltemp(j+1,:)=realpath(i,1:3)+j*buchang(i,:);
        end
        realpath_all((i-1)*(chazhi_num+1)+1:i*(chazhi_num+1),1:3)=realpath_alltemp;
    end
    realpath_all(end,1:3)=realpath(realpath_size,1:3);
    realpath_all_num=[1:(chazhi_num+1):size(realpath_all,1)]';
    realpath_all(:,4)=round((realpath_all(:,1)-Bounding(1,1))*sizeyuanshi)+1;%经度对应y
    realpath_all(:,5)=round((realpath_all(:,2)-Bounding(1,2))*sizeyuanshi)+1;%纬度对应x
    realpath_all(:,6)=demyuanshi(realpath_all(:,5)+(realpath_all(:,4)-1)*size(demyuanshi,1));
    realpath_all(:,7)=distance(realpath_all(:,2),realpath_all(:,1),realpath_all([1,1:end-1],2),realpath_all([1,1:end-1],1))/180*pi*6371004;
    realpath_all(1,8)=0;%第一个点的距离为0
    for i=2:size(realpath_all,1)
        realpath_all(i,8)=realpath_all(i-1,8)+realpath_all(i,7);
    end
    gaoducha_all=realpath_all(:,3)-realpath_all(:,6);
    jisuanTF.zonghangji=realpath_all(end,8);
    jisuanTF.maxhight=max(gaoducha_all);
    jisuanTF.minhight=min(gaoducha_all);
    jisuanTF.meanhight=mean(gaoducha_all);
    jisuanTF.tiedibili=length(find(gaoducha_all<=feijixianzhi.MaxTFlidigaodu&gaoducha_all>=feijixianzhi.MinTFlidigaodu-0.00001))/length(realpath_all(:,6))*100;
end


%% 高度后处理函数
%高度后处理
%输出：原始地形图、航点、地图边界、原始地图大小、飞机限制
%输出：航点对应飞机高度
function realpath_gaodu=gaodu(demyuanshi,realpath,Bounding,sizeyuanshi,feijixianzhi)
    realpath=realpath(:,[2,1]);%经纬换纬经
    %补充中间点
    chazhi_num=10;%两个点中间插入10个点
    buchang=(realpath(2:end,:)-realpath(1:end-1,:))/(chazhi_num+1);
    realpath_all=zeros((size(realpath,1)-1)*(chazhi_num+1)+1,5);
    realpath_alltemp=zeros(chazhi_num+1,2);
    for i=1:size(realpath,1)-1
        for j=0:chazhi_num
            realpath_alltemp(j+1,:)=realpath(i,:)+j*buchang(i,:);
        end
        realpath_all((i-1)*(chazhi_num+1)+1:i*(chazhi_num+1),1:2)=realpath_alltemp;
    end
    realpath_all(end,1:2)=realpath(end,:);
    realpath_all_num=[1:(chazhi_num+1):size(realpath_all,1)]';
    %经纬度转坐标
    [realpath_all_zb(:,1),realpath_all_zb(:,2)]=jwd2zb(realpath_all(:,1),realpath_all(:,2),Bounding,sizeyuanshi);
    %第三列为原始地形高
    realpath_all(:,3)=demyuanshi(realpath_all_zb(:,1)+(realpath_all_zb(:,2)-1)*size(demyuanshi,1));
    %第四列为每个点与上个点之间的距离
    realpath_all(:,4)=distance(realpath_all(:,1),realpath_all(:,2),realpath_all([1,1:end-1],1),realpath_all([1,1:end-1],2),6371004);
    realpath(:,3)=distance(realpath(:,1),realpath(:,2),realpath([1,1:end-1],1),realpath([1,1:end-1],2),6371004);
    %第五列为起点到每个点一共走过了多远（米）
    realpath_all(1,5)=0;%第一个点的距离为0
    for i=2:size(realpath_all,1)
        realpath_all(i,5)=realpath_all(i-1,5)+realpath_all(i,4);
    end
    %第六列为飞机高度
    realpath_gaodu=flighthigh(realpath_all(:,3),realpath_all(:,5),realpath(:,3),feijixianzhi,realpath_all_num);
end

%处理飞机高度
function planehigh=flighthigh(allhight,allsteplong,steplong,feijixianzhi,realpathallnum)
    Maxpashengjiao=tan(deg2rad(feijixianzhi.Maxpashengjiao));
    allhight=allhight+feijixianzhi.MinTFlidigaodu;%先都加14米
    for i=1:size(realpathallnum,1)-1%先遍历一遍
        allhight(realpathallnum(i):realpathallnum(i+1))=gaoduyuchuli(allsteplong(realpathallnum(i):realpathallnum(i+1)),allhight(realpathallnum(i):realpathallnum(i+1)));
    end
    planehigh=allhight(realpathallnum);%所有航迹点中抽出输出航迹点
    [~,index]=sort(planehigh,1,'descend');%对所有的高度从大到小排序，index是原来数组中的位置
    if size(index,1)>1
        for j=1:size(index,1)%遍历从高到低
            if index(j)==1%起点
                if planehigh(index(j))>planehigh(index(j)+1)%如果比下一个点高，就要判断下一个点满不满足要求
                    if planehigh(index(j)+1)<planehigh(index(j))-steplong(index(j)+1)*Maxpashengjiao%如果下一个点比当前点的最低允许高度低，就要提高下一个点
                        planehigh(index(j)+1)=planehigh(index(j))-steplong(index(j)+1)*Maxpashengjiao;
                        [~,index]=sort(planehigh,1,'descend');%重新排序，但是因为调整的一定是还没遍历到的点所以不影响循环
                    end
                end
            elseif index(j)==size(index,1)%终点
                if planehigh(index(j))>planehigh(index(j)-1)%如果比前一个点高，就要判断前一个点满不满足要求
                    if planehigh(index(j)-1)<planehigh(index(j))-steplong(index(j))*Maxpashengjiao%如果前一个点比当前点的最低允许高度低
                        planehigh(index(j)-1)=planehigh(index(j))-steplong(index(j))*Maxpashengjiao;%就要提高前一个点
                        [~,index]=sort(planehigh,1,'descend');%重新排序，但是因为调整的一定是还没遍历到的点所以不影响循环
                    end
                end
            else
                if planehigh(index(j))>planehigh(index(j)-1)%如果比前一个点高，就要判断前一个点满不满足要求
                    if planehigh(index(j)-1)<planehigh(index(j))-steplong(index(j))*Maxpashengjiao%如果前一个点比当前点的最低允许高度低，就要提高前一个点
                        planehigh(index(j)-1)=planehigh(index(j))-steplong(index(j))*Maxpashengjiao;
                        [~,index]=sort(planehigh,1,'descend');%重新排序，但是因为调整的一定是还没遍历到的点所以不影响循环
                    end
                end
                if planehigh(index(j))>planehigh(index(j)+1)%如果比下一个点高，就要判断下一个点满不满足要求
                    if planehigh(index(j)+1)<planehigh(index(j))-steplong(index(j)+1)*Maxpashengjiao%如果下一个点比当前点的最低允许高度低，就要提高下一个点
                        planehigh(index(j)+1)=planehigh(index(j))-steplong(index(j)+1)*Maxpashengjiao;
                        [~,index]=sort(planehigh,1,'descend');%重新排序，但是因为调整的一定是还没遍历到的点所以不影响循环
                    end
                end    
            end
        end
    end
end

%预处理，去掉中间高的
function  allhight=gaoduyuchuli(allsteplong,allhight)
    x1=allsteplong(1);
    y1=allhight(1);
    x2=allsteplong(end);
    y2=allhight(end);
    k=(y2-y1)/(x2-x1);
    b=y1-k*x1;
    ally=allhight-k*allsteplong-b;
    dayuy=find(ally>0);
%     dayuzuo=find(allhight(dayuy)>y1);
%     dayuyou=find(allhight(dayuy)>y2);  
    if ~isempty(dayuy)
%         dayu=ally(dayuy);
        y1=y1+max(ally(dayuy));
        y2=y2+max(ally(dayuy));
        k=(y2-y1)/(x2-x1);
        b=y1-k*x1;
        allhight=k*allsteplong+b;
    end
end

%% 最关键部分，改进A*算法
%输出经纬度（可以有高度），输入地图大小、灰度权重、、角度图、开始经纬度、终点经纬度
function [realpath,realpath_size,count]=TFTA_AstarFunction_new(juli_model,anglegray,Bounding,mapsize,beginlat,beginlng,endlat,endlng,feijixianzhi)
    [beginlatx,beginlngy]=jwd2zb(beginlat,beginlng,Bounding,mapsize);
    [endlatx,endlngy]=jwd2zb(endlat,endlng,Bounding,mapsize);
    CurrNode1=beginlatx;
    CurrNode2=beginlngy;
    realpath_length=round(abs(endlat-beginlat+endlng-beginlng)*mapsize)*50;
    realpath=zeros(realpath_length,2);
    ttt=juli(juli_model,Bounding,2,1,2,2);
    ggg=juli(juli_model,Bounding,2,1,1,1);
    ttgg=juli(juli_model,Bounding,2,2,1,1);
    %%
    %创建OpenNode表
    OpenNode.valuef=inf(size(anglegray,1),size(anglegray,2));
    OpenNode.valueg=inf(size(anglegray,1),size(anglegray,2));
    OpenNode.father=zeros(size(anglegray,1),size(anglegray,2));%每个点的父节点
    %%
    %寻路
    hxy=feijixianzhi.juliquanzhongH*juli(juli_model,Bounding,beginlngy,beginlatx,endlngy,endlatx);
    OpenNode.valuef(beginlatx,beginlngy)=hxy;
    OpenNode.valueg(beginlatx,beginlngy)=0;
    [min_rows,index_rows]=min(OpenNode.valuef,[],2);
    count=0;%调试用，计数器
    while(1)
        %将OpenNode表中目前代价最小的设置为当前点
        [min_rows(CurrNode1-1:CurrNode1+1,:),index_rows(CurrNode1-1:CurrNode1+1,:)]=min(OpenNode.valuef(CurrNode1-1:CurrNode1+1,:),[],2);
        [~,index_all]=min(min_rows);
        CurrNode1=index_all;
        CurrNode2=index_rows(index_all);
        GCurr=OpenNode.valueg(CurrNode1,CurrNode2);
        OpenNode.valuef(CurrNode1,CurrNode2)=NaN;
        %如果当前点就是终点，就表示已找到路径
        if CurrNode1==endlatx&&CurrNode2==endlngy 
            findway=1;%找到路径
            break;%循环结束
        end
        %%
        %对八邻域进行处理get_neighbors
        a_change=[-1 0 1 0 -1 1 1 -1];
        b_change=[0 -1 0 1 1 -1 1 -1];
        w_change=[ttt ggg ttt ggg ttgg ttgg ttgg ttgg];
        valuef_temp=OpenNode.valuef(CurrNode1-1:CurrNode1+1,CurrNode2-1:CurrNode2+1);
        valueg_temp=OpenNode.valueg(CurrNode1-1:CurrNode1+1,CurrNode2-1:CurrNode2+1);
        father_temp=OpenNode.father(CurrNode1-1:CurrNode1+1,CurrNode2-1:CurrNode2+1);
        Cfather=CurrNode1+(CurrNode2-1)*size(anglegray,1);
        for i=1:8%通过这八个值计算得到a，b，w的八种组合
            a=a_change(i)+2;
            b=b_change(i)+2;
            a_Curr=CurrNode1+a_change(i);%考虑点的X坐标
            b_Curr=CurrNode2+b_change(i);%考虑点的Y坐标
            w=feijixianzhi.juliquanzhongG*w_change(i)+double(anglegray(a_Curr,b_Curr))*feijixianzhi.poduquanzhong;%当前点到考虑点花费的代价（包含梯度）
            %% 可以在这里调整距离权重项F_h提高搜索效率！
            F_h = feijixianzhi.juliquanzhongH*juli(juli_model,Bounding,b_Curr,a_Curr,endlngy,endlatx);
            %% 要判断的点有没有超范围
            ifvalue=a_Curr>1&&a_Curr<size(anglegray,1)&&b_Curr>1&&b_Curr<size(anglegray,2)&&anglegray(a_Curr,b_Curr)~=255;
            if ifvalue&&~isnan(valuef_temp(a,b))
                if valuef_temp(a,b)==inf
                    valueg_temp(a,b)=GCurr+w;
                    valuef_temp(a,b)=valueg_temp(a,b)+F_h;
                    father_temp(a,b)=Cfather;
                    %如果在表里就把他现在代价与原来代价比较
                else
                    gxy=GCurr+w;
                    fxy=gxy+F_h;
                    if valuef_temp(a,b)>fxy%如果新的代价小，那就把小的赋值，新的代价大就不变
                        valuef_temp(a,b)=fxy;
                        valueg_temp(a,b)=gxy;
                        father_temp(a,b)=Cfather;
                    end                                                         
                end
            end
        end
        OpenNode.valuef(CurrNode1-1:CurrNode1+1,CurrNode2-1:CurrNode2+1)=valuef_temp;
        OpenNode.valueg(CurrNode1-1:CurrNode1+1,CurrNode2-1:CurrNode2+1)=valueg_temp;
        OpenNode.father(CurrNode1-1:CurrNode1+1,CurrNode2-1:CurrNode2+1)=father_temp;
       %%
        %OpenNode表空了，没有路径可以到
        count=count+1;%标记遍历几次循环，可删掉，调试用
%         if count>36000000
%             findway=0;
%             break;
%         end
    end
    %如果找到路径，从终点根据父节点追溯起点打印路径
    if findway==1
        realpath_size=1;
        pathtemp=zeros(1,2,size(realpath,1));%创建一个cell数组，存放路径
        CurrFather=OpenNode.father(CurrNode1,CurrNode2);
        while(1)%循环是因为不知道一共有多少路径点
            pathtemp(:,:,realpath_size)=[CurrNode1,CurrNode2];%从第一个开始将当前点加到路径里
            if CurrNode1==beginlatx&&CurrNode2==beginlngy%如果当前点已经是起点就跳出循环
                break;
            end
            NextNode1=mod(CurrFather,size(anglegray,1));%下一个点赋值
            NextNode2=(CurrFather-NextNode1)/size(anglegray,1)+1;
            NextFather=OpenNode.father(CurrFather);
            %下面是去点操作（待讨论在不考虑二值图的情况下如何实现）
%             [NextNode1,NextNode2,NextFather]=qudian(CloseNode,CurrNode1,CurrNode2,CurrFather);
            %去点循环结束，开始下一个点            
            CurrNode1=NextNode1;%把当前点换成下一个点，继续循环
            CurrNode2=NextNode2;
            CurrFather=NextFather;
            if realpath_size>=size(realpath,1)
                msgbox('超出路径数组限制');
                break;
            end
            realpath_size=realpath_size+1; 
        end
        pathxy=reshape(pathtemp,2,[])';%将找到的路径数组输出为N行2列的矩阵
        pathxy(1:realpath_size,:)=pathxy(realpath_size:-1:1,:);
        y_diving=cos(deg2rad(Bounding(2,2)));
        %根据限制角度改变路径的转弯半径大的点，遍历一次从头到尾调整一次（后期可改成直到没有大于限制的角）
        pathxy(1:realpath_size,:)=turnangle(pathxy(1:realpath_size,:),feijixianzhi.MaxAngle,y_diving);
        [realpath(1:realpath_size,1),realpath(1:realpath_size,2)]=zb2jwd(pathxy(1:realpath_size,1),pathxy(1:realpath_size,2),Bounding,mapsize);%路径坐标转成经纬度，第一列纬度，第二列经度
        realpath=realpath(:,[2,1]);
    else %如果没有找到路径3
        realpath=[];
        realpath_size=0;
        msgbox('没有找到路径');
    end
end
%%
%转弯角度
%判断转弯半径
function realpath=turnangle(realpath,MaxAngle,y_divlng)
    realpath(:,2)=realpath(:,2)*y_divlng;
    while(1)
        anglexy_all=lineangle3(realpath(1:end-2,1),realpath(1:end-2,2),realpath(2:end-1,1),realpath(2:end-1,2),realpath(3:end,1),realpath(3:end,2));
        find_bigangle=find(anglexy_all>MaxAngle+0.00001);
        if isempty(find_bigangle)
            break;
        else
            for i=1:size(find_bigangle,1)%遍历所有的相交路径
                %计算两条路径之间的角度
                anglexy=lineangle3(realpath(find_bigangle(i),1),realpath(find_bigangle(i),2),realpath(find_bigangle(i)+1,1),realpath(find_bigangle(i)+1,2),realpath(find_bigangle(i)+2,1),realpath(find_bigangle(i)+2,2));
                %角度大于限制角度的话，改变中间点
                if anglexy>MaxAngle
                    %将中间点替换
                    [realpath(find_bigangle(i)+1,1),realpath(find_bigangle(i)+1,2)]=changepoint(realpath(find_bigangle(i),1),realpath(find_bigangle(i),2),realpath(find_bigangle(i)+1,1),realpath(find_bigangle(i)+1,2),realpath(find_bigangle(i)+2,1),realpath(find_bigangle(i)+2,2),MaxAngle);
                end
            end
        end
    end
    realpath(:,2)=realpath(:,2)/y_divlng;
end
%给定三个点,求角度限值内的最接近的点，在一、三点的中垂线上找到角度限值上的点
function [x,y]=changepoint(x1,y1,x2,y2,x3,y3,anglelim)
    Beita=atan(abs((y3-y1)/(x3-x1)));%一、三点的连线与X轴的夹角
    m=tan(anglelim/360*pi)*sqrt((y3-y1)^2+(x3-x1)^2)/2;%中垂线上一、三中点到改变点的距离
    x4=(x1+x3)/2;%一、三中点X坐标
    y4=(y1+y3)/2;%一、三中点Y坐标
    x=x4+sign(x2-x4)*m*sin(Beita);%改变点的x坐标
    y=y4+sign(y2-y4)*m*cos(Beita);%改变点的y坐标
end
%求三个点两条线夹角
function anglexy=lineangle3(x1,y1,x2,y2,x3,y3)
    anglexy=acosd(dot([x1-x2,y1-y2],[x2-x3,y2-y3],2)./(((x1-x2).^2+(y1-y2).^2).^0.5.*((x2-x3).^2+(y2-y3).^2).^0.5));
end

%% 地图预处理函数
%地图处理部分函数调用
function [Bounding,anglegray,mapsize,sizeyuanshi]=TFTA_makebin(demyuanshi,map_lng,map_lat,map_scale,feijixianzhi,chazhi_model,chafen_model)
    %%
    sizeyuanshi=3600/map_scale;
    Bounding=[map_lng,map_lat-size(demyuanshi,1)/sizeyuanshi;map_lng+size(demyuanshi,2)/sizeyuanshi,map_lat];
    %%
    %矩阵分辨率调整
    Cutnum=111000.0/feijixianzhi.buchang/sizeyuanshi;
    switch chazhi_model
        case 1
            dem=dem_double(demyuanshi,Cutnum);
        case 2
            dem=dem_23(demyuanshi,Cutnum);
        case 3
            dem=dem_23_chizhi(demyuanshi,Cutnum);
    end
    mapsize=sizeyuanshi*Cutnum;
    %%
    divLat = 111000.0 / (mapsize-1);%每个高程格纬度上的实际距离
    divLng = divLat * cos(deg2rad(Bounding(1,2)));%每个高程格经度上的实际距离
    angle1=chafen(dem,divLng,divLat,chafen_model);
    anglegray=round((angle1-feijixianzhi.minpoduangle)/(feijixianzhi.maxpoduangle-feijixianzhi.minpoduangle)*254);%灰度图计算：通过0-254内对应最小角度到最大角度
    anglegray(anglegray<0)=0;%小于minangle都为0
    anglegray(anglegray>254)=254;%大于maxangle都设置为254
    anglegray=uint8(anglegray);%灰度图转换为uint8存储节省空间
end

%% 差分算法
function SSS=chafen(A,divLng,divLat,chafen_model)
    A=double(A);
    %八邻域
    A1=A([1,1:end-1],[1,1:end-1]);
    A2=A([1,1:end-1],:);
    A3=A([1,1:end-1],[2:end,end]);
    A4=A(:,[2:end,end]);
    A5=A([2:end,end],[2:end,end]);
    A6=A([2:end,end],:);
    A7=A([2:end,end],[1,1:end-1]);
    A8=A(:,[1,1:end-1]);
    %fg与ft计算
    switch chafen_model
        case 1
            fg_CZ=(A-A4)/divLng;
            ft_CZ=(A-A2)/divLat;
        case 2
            fg_CZ=(A4-A8)/(2*divLng);
            ft_CZ=(A2-A6)/(2*divLat);
        case 3
            fg_CZ=(A1-A3+2*(A8-A4)+A7-A5)/(8*divLng);
            ft_CZ=(A5-A3+2*(A6-A2)+A7-A1)/(8*divLat);
        case 4
            fg_CZ=(A1-A3+2^(1/2)*(A8-A4)+A7-A5)/((4+2*2^(1/2))*divLng);
            ft_CZ=(A5-A3+2^(1/2)*(A6-A2)+A7-A1)/((4+2*2^(1/2))*divLat);
        case 5
            fg_CZ=(A1-A3+A8-A4+A7-A5)/(6*divLng);
            ft_CZ=(A5-A3+A6-A2+A7-A1)/(6*divLat);
        case 6
            fg_CZ=(A1-A3+A7-A5)/(4*divLng);
            ft_CZ=(A5-A3+A7-A1)/(4*divLat);
    end
    %坡度值（转角度）
    SSS=rad2deg(atan((fg_CZ.^2+ft_CZ.^2).^(1/2)));
end

%% 经纬度转坐标,横坐标，纵坐标，纬度，经度，地图起始纬度，起始经度，地图大小(1200还是3600)
function [x,y]=jwd2zb(lat,lng,Bounding,mapsize)
    x=round((lat-Bounding(1,2))*mapsize)+1;
    y=round((lng-Bounding(1,1))*mapsize)+1;
end

%% 计算距离，model、经、纬、经、纬
function dis=juli(juli_model,Bounding,A,B,C,D)
    switch juli_model
        case 1
            %欧几里得距离
            dis=((cos(deg2rad(Bounding(1,2)))*(A-C))^2+(B-D)^2)^(1/2);
        case 2
            %曼哈顿距离
            dis=cos(deg2rad(Bounding(1,2)))*abs(A-C)+abs(B-D);
        case 3
            %修正版对角线距离
            dis=cos(deg2rad(Bounding(1,2)))*abs(A-C)+abs(B-D)+((1+(cos(deg2rad(Bounding(1,2))))^2)^0.5-(1+cos(deg2rad(Bounding(1,2)))))*min(cos(deg2rad(Bounding(1,2)))*abs(A-C),abs(B-D));
        case 4
            %切比雪夫距离
            dis=max(cos(deg2rad(Bounding(1,2)))*abs(A-C),abs(B-D));
    end
end

%% 二维三次卷积插值算法
function g1=dem_23_chizhi(f,k)                             
    [m,n]=size(f);  
    a=f(1,:);
    c=f(m,:) ;        %将图像扩展四行四列
    b=[f(1,1),f(1,1),f(:,1)',f(m,1),f(m,1)];
    d=[f(1,n),f(1,n),f(:,n)',f(m,n),f(m,n)];
    a1=[a;a;f;c;c];
    b1=[b;b;a1';d;d];
    f=b1';
    f1=double(f);
    for i=1:m*k
        dx=i/k-floor(i/k);           
         x=floor(i/k)+2;   
         A=[sw(1+dx) sw(dx) sw(1-dx) sw(2-dx)];   %四个横坐标的权重W(i)
      for j=1:n*k
        dy=rem(j,k)/k;
         y=floor(j/k)+2;   
        C=[sw(1+dy);sw(dy);sw(1-dy);sw(2-dy)]; 
        B=[f1(x-1,y-1),f1(x-1,y) ,f1(x-1,y+1), f1(x-1,y+2)    
             f1(x ,y-1),f1(x,   y),f1(x,  y+1),f1(x,   y+2)
             f1(x+1,y-1),f1(x+1,y),f1(x+1,y+1),f1(x+1,y+2)
             f1(x+2,y-1),f1(x+2,y),f1(x+2,y+1),f1(x+2,y+2)];
        g1(i,j)=(A*B*C);
      end
    end
end

function A=sw(w1)
    w=abs(w1);
    a=-0.5;
    if w<1&&w>=0
      A=1-(a+3)*w^2+(a+2)*w^3;
    else if w>=1&&w<2
      A=a*w^3-5*a*w^2+(8*a)*w-4*a;
    else
      A=0;
        end
    end
end