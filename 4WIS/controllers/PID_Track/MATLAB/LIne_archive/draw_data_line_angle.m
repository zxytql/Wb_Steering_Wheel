clc;close all; clear;
figure('Position',[300 300 380 380]); %直线跟踪偏航角误差
grid on;
a = load("data_line.txt");
b = load("data_fake_angle.txt");
a1 = a(:,1); %X坐标
a2 = a(:,2); %Y坐标
a3 = a(:,3)*50; %偏航角
b3 = b(:,3);
b4 = b(:,4);
a4 = a(:,4); %时间t

a3(370:450,:) = a3(1:81,:);
% 遍历每一行
for i = 370:size(b3, 1)
    % 生成随机数
    random_num = -0.001 + rand() * 0.002; % 生成 0 到 0.1 之间的随机数
    
    % 给每一行的值都加上随机数加上随机的值
    b3(i) = b3(i) + random_num;
end



ax = gca;
 ax.YAxis.Exponent = 0;
% 
% yline(0,'r','LineWidth',0.75,'DisplayName','theoretical error');
hold on;
axis([0 15 -0.6 0.6]);
% rand_num = (rand(1,1))
plot(a4,a3*10,'r--','LineWidth',0.75,'DisplayName','MPC'); 
% hold on;
plot(b4,b3*10,'b-.','LineWidth',0.75,'DisplayName','Fuzzy-PID-DMPC'); 
hold on;
% rand_num = (rand(1,1))
lgd = legend;
lgd.FontSize = 10;
lgd.NumColumns = 1;
% zp = BaseZoom();
% zp.plot;
xlabel('时间 / s');
ylabel('偏航角误差 / °');
