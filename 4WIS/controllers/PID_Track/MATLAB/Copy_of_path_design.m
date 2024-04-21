%% design path from points by spline
close all; clear;
% point = [0, 0;
%     1, 0;
%     2,0
%     3,0.5
%     4,1.5
%     4.8, 1.5
%     5,0.8
%     6, 0.5
%     6.5, 0
%     7.5, 0.5
%     7,2
%     6, 3
%     5, 4
%     4., 2.5
%     3, 3
%     2., 3.5;
%     1.3, 2.2
%     0.5, 2.
%     0,3];

point = [0, 0;
    0, 1;
    0,2
    0.5,3
    1.5,4
    1.5,4.8
    0.8,5
    0.5,6
    0,6.5
    0.5,7.5
    2,7
    3,6
    3,5
    2.5,4
    3, 3
    3.5,2;
    2.2,1.3
    2,0.5
    3,0];

s = 1:1:length(point);


px_spline = spline(s, point(:,1), 1:0.0197:length(point));
py_spline = spline(s, point(:,2), 1:0.0197:length(point));

p_spline = [(px_spline*1000)', (py_spline*1000)'];



%% insert yaw

% yaw = zeros(length(px_spline), 1);
% for i = 2:length(px_spline)-1
%     x_forward = px_spline(i+1);
%     x_backward = px_spline(i-1);
%     y_forward = py_spline(i+1);
%     y_backward = py_spline(i-1);
%     yaw(i) = atan2(y_forward-y_backward, x_forward-x_backward);
% end
% yaw(1) = yaw(2);
% yaw(end) = yaw(end-1);


%% plot with attitude

% arrow_scale = 0.01;
% set(gcf, 'Position', [100 100 260 220]); 

a = load("1.txt");
a1 = a(:,1);
a2 = a(:,2);
a3 = a(:,3);
b = load("2.txt");
b1 = b(:,1);
b2 = b(:,2);
plot(-a1, a2+5776,'k--','LineWidth',2,'DisplayName','Ref'); 
grid on;
hold on;
plot(b1-250,b2,'b','LineWidth',2,'DisplayName','FDMPC'); hold on;
% plot(a1,a2,'r-.','LineWidth',2,'DisplayName','MPC'); 

array = (1:73)';
figure(2);
yline(87,'LineWidth',2); hold on;
plot(array,a3,'r--','LineWidth',2);

hold off;
xlabel('X (mm)');
ylabel('Y (mm)');
% lgd = legend;
% lgd.FontSize = 12;
% lgd.NumColumns = 1;
zp = BaseZoom(); % 2570 4280 
% zp.plot;
% zp.plot;

figure('Position',[320 320 523 437]);
% nearest_point_x = zeros(size(a1,1),1);
% nearest_point_y = zeros(size(a1,1),1);
% err = zeros(size(a1,1),1);
% for i = 1:size(a1,1)
%     dist = sqrt((p_spline(:,1)-a1(i,1)).^2 + (p_spline(:,2)-a2(i,1)).^2);
%     [~, min_dist_idx] = min(dist)
%     nearest_point_x(i) = p_spline(min_dist_idx,1);
%     nearest_point_y(i) = p_spline(min_dist_idx,2);
%     err(i) = (sqrt((a1(i,1)-nearest_point_x(i))^2 + (a2(i,1)-nearest_point_y(i))^2));
% end

%求横向误差
% dist = sqrt((p_spline(:,1)-a1(:,1)').^2 + (p_spline(:,2)-a2(:,1)').^2);
% [~, min_dist_idx] = min(dist,[],1);
% nearest_point_x = p_spline(min_dist_idx,1);
% nearest_point_y = p_spline(min_dist_idx,2);
% err = sqrt((a1-nearest_point_x).^2 + (a2-nearest_point_y).^2);
% axis([0 20 -10 10]); hold on;

nearest_point_x = zeros(size(a1,1),1);
nearest_point_y = zeros(size(a1,1),1);
err = zeros(size(a1,1),1);
normalized_err_1 = zeros(size(a1,1),1);

for i = 1:size(a1,1)
    dist = sqrt((p_spline(:,1)-a1(i,1)).^2 + (p_spline(:,2)-a2(i,1)).^2);
    [~, min_dist_idx] = min(dist);
    nearest_point_x(i) = p_spline(min_dist_idx,1);
    nearest_point_y(i) = p_spline(min_dist_idx,2);
    vec1 = [nearest_point_x(i) - p_spline(max(min_dist_idx-1,1),1); nearest_point_y(i) - p_spline(max(min_dist_idx-1,1),2)];
    vec2 = [p_spline(min(min_dist_idx+1,size(p_spline,1)),1) - nearest_point_x(i); p_spline(min(min_dist_idx+1,size(p_spline,1)),2) - nearest_point_y(i)];
    cross_prod = vec1(1)*vec2(2) - vec1(2)*vec2(1);
    err(i) = sign(cross_prod) * sqrt((a1(i,1)-nearest_point_x(i))^2 + (a2(i,1)-nearest_point_y(i))^2);
    normalized_err_1(i) = (err(i) - 6) / 12; %归一化
end

axis([0 20 -10 10]); hold on;
yline(0,'k','LineWidth',1.5);
plot(a4,normalized_err_1,'b','LineWidth',1.5,'DisplayName','FDMPC'); hold on;
avg_1 = sum(normalized_err_1)/a4

nearest_point_x = zeros(size(b1,1),1);
nearest_point_y = zeros(size(b1,1),1);
err = zeros(size(b1,1),1);
normalized_err_2 = zeros(size(b1,1),1);

for i = 1:size(b1,1)
    dist = sqrt((p_spline(:,1)-b1(i,1)).^2 + (p_spline(:,2)-b2(i,1)).^2);
    [~, min_dist_idx] = min(dist);
    nearest_point_x(i) = p_spline(min_dist_idx,1);
    nearest_point_y(i) = p_spline(min_dist_idx,2);
    vec1 = [nearest_point_x(i) - p_spline(max(min_dist_idx-1,1),1); nearest_point_y(i) - p_spline(max(min_dist_idx-1,1),2)];
    vec2 = [p_spline(min(min_dist_idx+1,size(p_spline,1)),1) - nearest_point_x(i); p_spline(min(min_dist_idx+1,size(p_spline,1)),2) - nearest_point_y(i)];
    cross_prod = vec1(1)*vec2(2) - vec1(2)*vec2(1);
    err(i) = sign(cross_prod) * sqrt((b1(i,1)-nearest_point_x(i))^2 + (b2(i,1)-nearest_point_y(i))^2);
    normalized_err_2(i) = (err(i) - 2) / 4; %归一化
end

axis([0 20 -10 10]); hold on;
xlabel('Time (s)');
ylabel('Lateral error (mm)');
lgd = legend;
lgd.FontSize = 12;
lgd.NumColumns = 1;

plot(b4,normalized_err_2,'r--','LineWidth',1.5,'DisplayName','MPC'); hold off;
avg_2 = sum(normalized_err_2)/b4

% 计算曲率值
dx = diff(p_spline(:, 1)/1000);
dy = diff(p_spline(:, 2)/1000);

d2x = diff(dx);
d2y = diff(dy);
dx = dx(1:end-1);
dy = dy(1:end-1);
curvature = abs(d2y.*dx - d2x.*dy) ./ (dx.^2 + dy.^2).^1.5;
curvature = [0; 0; curvature];

% 绘制曲率图
figure('Position',([400 400 523 437]));

tiledlayout(2,1)
ax1 = nexttile;
plot(ax1,1:length(curvature), curvature);
xlabel('Index of trajectory points');
ylabel('Curvature (m^{-1})');
% 
ax2 = nexttile; %P_min = 5 P_max = 15

% % 假设 curvature 是已知的曲率序列
curvature_trend = smooth(curvature); % 平滑曲线趋势
curvature_scaled = round(rescale(curvature_trend, 5, 15)); % 将曲率趋势缩放到 [5, 15] 区间并四舍五入为整数
plot(ax2,1:length(curvature),curvature_scaled);
xlabel('Index of trajectory points');
ylabel('P');

%% save path
% path = [(px_spline*1000)', (py_spline*1000)'];
% save('path.txt', 'path','-ascii');
% dlmwrite('path1.txt',path);