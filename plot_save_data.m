figure(123)
clf;
hold on
grid on
box on
xlim tight
plot(save_data(:,1),save_data(:,5))
plot(save_data(:,1),save_data(:,6))
plot(save_data(:,1),save_data(:,7))
plot(save_data(:,1),sqrt(save_data(:,5).^2+save_data(:,6).^2+save_data(:,7).^2))
legend('u','v','w','U')
set(gca,'FontSize',16) 
set(gcf,'color','white');
xlabel('Time [s]')
ylabel('surge [u]')

figure(126);clf;
hold on
grid on
box on
xlim tight
plot(save_data(:,1),save_data(:,8))
plot(save_data(:,1),save_data(:,11))
set(gca,'FontSize',16) 
set(gcf,'color','white');
xlabel('Time [s]')
ylabel('phi [r]')
legend('actual','command')

figure(127);clf;
hold on
grid on
box on
xlim tight
plot(save_data(:,1),save_data(:,9))
plot(save_data(:,1),save_data(:,12))
set(gca,'FontSize',16) 
set(gcf,'color','white');
xlabel('Time [s]')
ylabel('theta [r]')
legend('actual','command')

figure(128);clf;
hold on
grid on
box on
xlim tight
plot(save_data(:,1),save_data(:,10))
plot(save_data(:,1),save_data(:,13))
set(gca,'FontSize',16) 
set(gcf,'color','white');
xlabel('Time [s]')
ylabel('psi [r]')
legend('actual','command')

figure(129);clf;
hold on
grid on
box on
xlim tight
plot(save_data(:,1),save_data(:,14))
set(gca,'FontSize',16) 
set(gcf,'color','white');
xlabel('Time [s]')
ylabel('Force')



