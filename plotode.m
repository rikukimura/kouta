figure(1)
plot(TOUT,VOUT(:,3),'LineWidth',1.5)
set(gca,'Fontname','Times New Roman','FontSize',14);
xlabel('time[s]','Fontname','Times New Roman','FontSize',14);
ylabel('{distance} [mm]','Fontname','Times New Roman','FontSize',14);
xlim([0 18])
ylim([0 110])
saveas(gcf,'sim-distance-rip-ode.svg')
%rdp t=12
%rip t=9
VOUT_m = VOUT(:,1)/1000;
VOUT_m(1465:end,1)=0;
VOUT_m(1244:end,1)=0;
VOUT_m(313:end,1)=0;
figure(2)
plot(TOUT,VOUT_m(:,1),'LineWidth',1.5)
set(gca,'Fontname','Times New Roman','FontSize',14);
xlabel('time[s]','Fontname','Times New Roman','FontSize',14);
ylabel('{velocity} [m/s]','Fontname','Times New Roman','FontSize',14);
xlim([0 18])
ylim([0 0.03])
saveas(gcf,'sim-velocity-rip-ode.svg')