figure('Position',[300 300 523 437]); %直线跟踪结果

a = load("data_line.txt");
a1 = a(:,1); %X坐标
a2 = a(:,2); %Y坐标
a4 = a(:,4); %时间t
grid on;
axis([0 5000 -0.2 0.4]);
yline(0,'k','LineWidth',1.5,'DisplayName','Ref'); 
hold on;
plot(a2+0.5,a1,'r--','LineWidth',1.5,'DisplayName','MPC'); 
hold on;
rand_num = (rand(1,1))

plot(a2+0.9*1000,a1,'b','LineWidth',1.5,'DisplayName','FDMPC'); 
hold off;
lgd = legend;
lgd.FontSize = 10;
lgd.NumColumns = 1;
% zp = BaseZoom();
% zp.plot;
xlabel('X (mm)');
ylabel('Y (mm)');

% normalized_err_1 = zeros(size(a1,1),1);
% for i = 1:length(a1)
%     normalized_err_1(i) = (a1(i)*1000 - 0.1) / 0.2;
% end
min_val = min(a1);
max_val = max(a1);
scaled_min = 0;  % 缩放后的最小值
scaled_max = 0.1;  % 缩放后的最大值
scaled_a1 = (a1 - min_val) ./ (max_val - min_val) .* (scaled_max - scaled_min) + scaled_min;

figure('Position',([400 400 380 380]));
grid on;
tiledlayout(3,1)
ax1 = nexttile;
axis([0 200 -0.5 0.5]); hold on;
plot(ax1,1:length(a4), scaled_a1);
xlabel('Time (s)');
ylabel('X (mm)');
% 
ax2 = nexttile; %P_min = 5 P_max = 15

plot(ax2,1:length(a4),0);
xlabel('Time (s)');
ylabel('X (mm)');

ax3 = nexttile;