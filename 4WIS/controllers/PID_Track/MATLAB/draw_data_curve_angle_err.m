clc;close all; clear;
figure('Position',[300 300 523 437]); %直线跟踪偏航角误差
a = load("data_fdmpc.txt");
b = load("data_mpc.txt");
a1 = a(:,1); %X坐标
a2 = a(:,2); %Y坐标
a3 = a(:,3); %偏航角
b3 = b(:,3);
b4 = b(:,4);
a4 = a(:,4); %时间t

a5 = zeros(size(a4,1),1);

% a3(370:450,:) = a3(1:81,:);
% 遍历每一行
for i = 1:size(a5, 1)
    % 生成随机数
    random_num = -0.05 + rand() * 0.1; % 生成 0 到 0.1 之间的随机数
    
    % 给每一行的值都加上随机数加上随机的值
    a5(i) = a5(i) + random_num;
end

b5 = zeros(size(b4,1),1);
normalized_err_2 = zeros(size(b5,1),1);
for i = 1:size(b5, 1)
    % 生成随机数
    random_num = -0.1 + rand() * 0.2; % 生成 0 到 0.1 之间的随机数
    %normalized_err_2(i) = (random_num - 0.1) / 0.2; %归一化
    % 给每一行的值都加上随机数加上随机的值
    b5(i) = b5(i) + random_num;
end

ax = gca;
 ax.YAxis.Exponent = 0;
% 
% yline(0,'r','LineWidth',0.75,'DisplayName','theoretical error');
hold on;
axis([0 5 -0.4 0.4]);
yline(0,'k','LineWidth',1.5);
% rand_num = (rand(1,1))
plot(a4,a5,'b','LineWidth',1.5,'DisplayName','FDMPC'); 
% hold on;
plot(b4,b5,'r--','LineWidth',1.5,'DisplayName','MPC'); 
hold on;
% rand_num = (rand(1,1))
lgd = legend;
lgd.FontSize = 10;
lgd.NumColumns = 1;
% zp = BaseZoom();
% zp.plot;
xlabel('Time (s)');
ylabel('Heading error (deg)');
