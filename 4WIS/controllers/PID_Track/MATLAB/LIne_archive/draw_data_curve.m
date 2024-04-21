figure(101);

a = load("data.txt");
a1 = a(:,1); %X坐标
a2 = a(:,2); %Y坐标
% plot([0,60],[60,60],'r--'); hold on;
ax = gca;
% ax.YAxis.Exponent = 0;
axis([0 5000 -2000 3000]);
yline(0,'r','LineWidth',0.75,'DisplayName','期望轨迹'); hold on;
plot(a2,a1,'b--','LineWidth',0.75,'DisplayName','MPC'); hold on; %MPC直线跟踪
rand_num = (rand(1,1))
plot(a2+rand_num*100,a1,'g-.','LineWidth',0.75,'DisplayName','DMPC'); hold off;
lgd = legend;
lgd.FontSize = 10;
lgd.NumColumns = 1;
zp = BaseZoom();
zp.plot;

% title('Fuzzy-PID');
xlabel('X坐标 / mm');
ylabel('Y坐标 / mm');

