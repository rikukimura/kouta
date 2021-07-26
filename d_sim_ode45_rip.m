clf;
close;
clear %ワークスペースの数値を初期化

ani = 0;%1 creates an animation
count2 = 0;

L =76;%paddle length  [mm]
Rh =50;%[mm]ヒンジ半径
radii = 56;
n = 1000;%分割数n
num_seg = 100 ;
m=0;
p=0;
rS =20 ;
% theta_w =zeros(n,1);
% centers = zeros(n,2);
% 
% xP = zeros(n,1);
% yP = zeros(n,1);
% xS = zeros(n,1);
% yS = zeros(n,1);
% xH = zeros(n,1);
% yH = zeros(n,1);
% xW = zeros(n,1);
% yW = zeros(n,1);
% 
% theta_pi = zeros(n,1);
% 
% seg_x = zeros(n,num_seg);
% seg_y = zeros(n,num_seg);
% G     = zeros(n,num_seg);
% B     = zeros(n,num_seg);
% z     = zeros(n,num_seg);
% Z     = zeros(n,num_seg);
% 
% alphaX = zeros(n,num_seg);
% alphaZ = zeros(n,num_seg);
% sigma_x = zeros(n,num_seg);
% sigma_z = zeros(n,num_seg);
% 
% ForceX = zeros(n,num_seg);
% ForceZ = zeros(n,num_seg);
% Dragforce =zeros(n,num_seg);
% 
% Re = zeros(n,num_seg);
% Cd = zeros(n,num_seg); % Drag coefficient
% 
% ForceX_sum = zeros(n,1);
% ForceZ_sum = zeros(n,1);
% DragX      = zeros(n,1);
% 
% Net_Fx = zeros(1,n);
% a      = zeros(1,n);
% v_x    = zeros(1,n);
% mov_x  = zeros(1,n);

% t = linspace(0,18,n);
% t = t.';

% w_x = zeros(n,2);
% w_x(1:n,2) = 56;
w_c = [0,56];%wheel center
Mass = 7.8; %[kg]
TreadWidth =63;%[mm]
scaleFactor = 1.1;
sand_d = 1480;% dencity of sand [kg/m^3]
sand_d_m = sand_d;
sand_d = sand_d *10^(-9); %[kg/mm^3]

myu = 5 *10 ^(10); % myu :粘度[Pa*s]
nyu =  myu   / sand_d_m; %動粘度[m^2/s]
nyu =  nyu * 10^(-6); %動粘度[mm^2/s]




A00 = 0.206 ;
A10 = 0.169 ;
B11 = 0.212 ;
B01 = 0.358 ;
Bm11 = 0.055;
C11 = -0.124;
C01 = 0.253 ;
Cm11 = 0.007;
D10 = 0.088 ;

g = 9.8;
seg_length = L / num_seg ;%length of one segment
dx(1,1:num_seg) = 0;
dy(1,1:num_seg) = 0;
d_vx(1,1:num_seg) = 0;
d_vy(1,1:num_seg) = 0;
theta_w(1) = 0;
%% Initialized
x = w_c(1);
y = w_c(2);
w_x(1) =0; %
w_x(1,2) = 56;

%% パドルの手先位置の移動速度Δvx,vyの計算
omega = deg2rad(10) ;%wheel angular speed [rad/s]
duration = deg2rad(180) / (omega) ; %test time
dt = duration / n ;
q = 1;
k =1;
v = 1;
v2 =1;
theta = (linspace(pi/2,0,500)).';%unit rad

init_velocity     = [0 0];
TireAxleInitCoord = [0 56];
dbforce = 0 ;
Cd = 1.7 ;
Vo = zeros(1,5);
count = 0 ;

Vo(1) = init_velocity(1);% initial velocity in x-dir
Vo(2) = init_velocity(2);% initial velocity in z-dir
Vo(3) = TireAxleInitCoord(1);% initial axle position in x
Vo(4) = TireAxleInitCoord(2);% initial axle position in z
Vo(5) = 0;% initial dissipated Power




options = odeset('RelTol',1e-2,'AbsTol',1e-1);
odefix = @(t,V) Function_rip(t,V,Mass,TreadWidth,seg_length,...
 omega,num_seg,scaleFactor,g,w_x,Rh,radii,w_c,L,sand_d,Cd,count);
[TOUT,VOUT] = ode45(odefix,[0 duration],Vo,options);


% for i=1:n
%     
%     xS(i) = w_x(i,1);
%     yS(i) = 56;
%     
%     xH(i,1) = (Rh * cos(theta_w(i,1)) + w_x(i,1));
%     yH(i) = (Rh * sin(theta_w(i,1)) + w_c(2));
%     xW(i,1) = (radii * cos(theta_w(i,1)) + w_x(i,1));%used for plot
%     yW(i,1) = (radii * sin(theta_w(i,1)) + w_c(2));
%     theta_w = theta_w + (- omega) * dt ;
%     theta_pi(i,1) =  atan2(yH(i) - yS(i),xH(i) - xS(i));
%     if  theta_pi(i,1) >0
%         theta_pi(i,1) = deg2rad(-180);
%     end
%     xP(i) = xS(i) + L*cos(theta_pi(i));
%     yP(i) = yS(i) + L*sin(theta_pi(i)); %パドル位置
%     
%     for q = 1:num_seg
%         seg_y(i,q) = yS(i) + (L*sin(theta_pi(i))/num_seg) * q  ;
%         seg_x(i,q) = xS(i) + (L*cos(theta_pi(i))/num_seg) * q  ;
%         if i >1
% %             d_vx(i,q) = (seg_x(i,q) - seg_x(i-1,q))/dt;
% %             d_vy(i,q) = (seg_y(i,q) - seg_y(i-1,q))/dt;
%             
%                       d_vx(i,q) = omega * (seg_y(i,q) - seg_y(i-1,q)) + v_x(1,i-1);
%                       d_vy(i,q) = -omega * (seg_x(i,q) - seg_x(i-1,q)) ;
%         end
%         
%         if -theta_pi(i,1) < pi/2
%             B(i,q) = -theta_pi(i,1)  ;
%         else
%             B(i,q) = -theta_pi(i,1)  ;
%         end
%         
%         if  d_vx(i,q) < 0
%             G(i,q) = atan(d_vy(i,q)./d_vx(i,q));
%         elseif d_vx(i,q) >= 0
%             G(i,q) = atan(d_vy(i,q)./d_vx(i,q)) +pi;
%         end
%         %       if  d_vx(i,q) < 0
%         %             G(i,q) = atan2(d_vy(i,q),d_vx(i,q));
%         %       elseif d_vx(i,q) >= 0
%         %             G(i,q) = atan2(d_vy(i,q),d_vx(i,q)) + pi;
%         %       end
%         
%         
%         z(i,q) = seg_y(i,q);
%         
%         Z(i,q) = z(i,q);
%         Z(Z>0) = 0;
%         
%         alphaX(i,q)=(Cm11 * cos(-2 * B(i,q) + G(i,q)) + C01 * cos(G(i,q)) + ...
%             C11 * cos(2 * B(i,q) + G(i,q)) + D10 * sin(2 * B(i,q)));
%         alphaZ(i,q) = (A10 * cos(2 * B(i,q)) + A00 + Bm11 * sin((-2 * B(i,q)) + G(i,q)) + ...
%             B01 * sin(G(i,q)) + B11 * sin((2 * B(i,q)) + G(i,q)));
%         
%         sigma_x(i,q) = alphaX(i,q) * -Z(i,q) *scaleFactor * 0.1 ;
%         sigma_z(i,q) = alphaZ(i,q) * -Z(i,q) *scaleFactor * 0.1 ;
%         
%         
%         Re(i,q) = d_vx(i,q) * seg_length *sin(B(i,q))  / nyu ;
%         Re(i,q) = abs(Re(i,q));
%         
%         %          if Re(i,q) < 12
%         %               Cd(i,q) = 1.328  / sqrt(Re(i,q));
%         %            elseif Re(i,q) > 12 && Re(i,q) < 2000
%         %               Cd(i,q) =  2.9 / (Re(i,q)^0.601);
%         %              else
%         %               Cd(i,q) = 2;
%         %          end
%         Cd(i,q) =1.7;
%         if Z(i,q) < 0
%             
%             ForceX(i,q) = sigma_x(i,q) *  TreadWidth * seg_length * 0.01  ;
%             ForceZ(i,q) = sigma_z(i,q) * TreadWidth * seg_length  * 0.01  ;
%             %             Dragforce(i,q) = 0.5 * (d_vx(i,q))^2 * sand_d * seg_length *sin(B(i,q))* TreadWidth * Cd(i,q)  ;
%             Dragforce(i,q) = 0.5 * (d_vx(i,q))^2 * sand_d * seg_length * TreadWidth * Cd(i,q)  ;
%         else
%             ForceX(i,q) = 0 ;
%             ForceZ(i,q) = 0 ;
%             Dragforce(i,q) =0;
%         end
%         
%         %          ForceX(i,q) = sigma_x(i,q) *  TreadWidth * -z(i,q) * 0.01 / 2 ;
%         %          ForceZ(i,q) = sigma_z(i,q) * TreadWidth * -z(i,q) * 0.01  / 2;
%         
%         
%         %           ForceX(i,q) = sigma_x(i,q) * seg_length * TreadWidth * -z(i,q) * 0.01 /2 ;
%         %           ForceZ(i,q) = sigma_z(i,q) * seg_length * TreadWidth * -z(i,q) * 0.01 /2;
%         
%         %           ForceX(i,q) = alphaX(i,q) * (xP(i)/num_seg) * TreadWidth ;
%         %           ForceZ(i,q) = alphaZ(i,q) * (yP(i)/num_seg) * TreadWidth ;
%     end
%     ForceX_sum(i) = sum(ForceX(i,1:end));
%     ForceZ_sum(i) = sum(ForceZ(i,1:end));
%     DragX(i)      = sum(Dragforce(i,1:end));
%     Net_Fx(i) =  ForceX_sum(i)-DragX(i);
%     a(i) = Net_Fx(i) / Mass ;%[m/s^2]
%     a(i) =  real(a(i)) ;
%     if i == 1
%         v_x(i) =  0;
%         mov_x(i) = 0;
%         w_x(i+1) = w_x(i);%次の車輪の中心位置座標[x][mm]
%         
%     else if a(i) ~= 0
%             v_x(i) = v_x(i-1)+a(i)*dt;
%             mov_x(i) = v_x(i-1)*dt + 1/2 *a(i)*dt*dt;
%             w_x(i+1) = mov_x(i) * 1000 + w_x(i); %change unit [m] to [mm]
%         else
%             v_x(i) = 0;
%             mov_x(i) = mov_x(i-1);
%             w_x(i+1) = w_x(i);
%         end
%     end
% end

%% Animation part
w_x(1,2) = 56;
if ani == 1
    fig=figure(10);
    set(gcf,'name','smple_anm');
    set(fig,'DoubleBuffer','on')
    set(gca,'Fontname','Times New Roman','FontSize',14);
    clf
    writeObj= VideoWriter('ani_dis_fp','MPEG-4');
    open(writeObj);
    for tdx= 1:n
        clf
        %           x_sand = [-80 w_x(tdx,1)+120 w_x(tdx,1)+120 -80];
        x_sand = [-80 450 450 -80];
        y_sand = [0 0 -50 -50];
        patch(x_sand,y_sand,'yellow','FaceAlpha',.2); %plot sand level
        center = [w_x(tdx,1) w_x(tdx,2)];
        viscircles(center,radii,'Color','k'); %draw wheel
        hold on
        plot([xS(tdx,1),xP(tdx,1)],[yS(tdx,1),yP(tdx,1)],'LineWidth',1.5) %draw paddle
        hold on
        plot(xW(tdx,1),yW(tdx,1),'-o','Color','g','Markersize',5,'MarkerFaceColor','g');
        hold on
        plot(xS(tdx,1),yS(tdx,1),'-o','Color','b','Markersize',5,'MarkerFaceColor','k');
        hold on
        plot(xP(1:tdx,end),yP(1:tdx,end),'-o','Color','b','Markersize',2.5,'MarkerFaceColor','b');
        daspect([1 1 1]);
        xlabel('{holizon} [mm]','Fontname','Times New Roman','FontSize',14);
        ylabel('{vertical} [mm]','Fontname','Times New Roman','FontSize',14);
        %           xlim([-80 w_x(tdx,1)+80]);
        xlim([-80 200]);
        ylim([-40 120]);
        drawnow;
        F=getframe(fig);
        writeVideo(writeObj,F);
    end
    close(writeObj);
end

%% plot area
% figure(1)
% plot(xS,yS,'k','LineWidth',1.5,'MarkerSize',100);
% % xlim([-50,50]);
% % ylim([0,100]);
% % daspect([1 1 1]);
% %
% % figure(2)
% % plot(xH,yH)
% % daspect([1 1 1]);
% %
% figure(3)
% plot(xP,yP);
% % daspect([1 1 1]);
% theta_plot = linspace(0,pi,n);
% 
% % figure(4)
% % plot(theta_plot,ForceX_sum,'k','LineWidth',1.5)
% %  hold on
% % plot(theta_plot,ForceZ_sum,'r','LineWidth',1.5)
% % xlim([0 pi])
% % ylim([-1 6])
% % xlabel('{\it\theta_{W}} [rad]','Fontname','Times New Roman','FontSize',14);
% % ylabel('{\itF} [N]','Fontname','Times New Roman','FontSize',14);
% % legend('F_{x}','F_{y}')
% % %  saveas(gcf,'sim-fp.svg')
% 
% % figure(5)
% % plot(t,a,'LineWidth',1.5)
% % xlabel('time[s]','Fontname','Times New Roman','FontSize',14);
% % ylabel('{\ita{x}} [m/s^2]','Fontname','Times New Roman','FontSize',14);
% 
% figure(6)
% plot(t,v_x,'LineWidth',1.5)
% xlabel('time[s]','Fontname','Times New Roman','FontSize',14);
% ylabel('{\itv_{x}} [m/s]','Fontname','Times New Roman','FontSize',14);
% 
% figure(7)
% plot(t,w_x(:,1),'LineWidth',1.5)
% set(gca,'Fontname','Times New Roman','FontSize',14);
% xlabel('time[s]','Fontname','Times New Roman','FontSize',14);
% ylabel('{distance} [mm]','Fontname','Times New Roman','FontSize',14);
% xlim([0 18])
% ylim([0 110])
% saveas(gcf,'sim-distance-fp.svg')
% 
% figure(8)
% plot(theta_plot,Net_Fx,'k','LineWidth',1.5)
% hold on
% plot(theta_plot,ForceZ_sum,'r','LineWidth',1.5)
% set(gca,'Fontname','Times New Roman','FontSize',14);
% xlim([0 pi])
% ylim([-1 6])
% xlabel('{\it\theta_{W}} [rad]','Fontname','Times New Roman','FontSize',14);
% ylabel('{\itF} [N]','Fontname','Times New Roman','FontSize',14);
% legend('F_{x}','F_{y}')
% saveas(gcf,'sim-force-fp.svg')
% 
% figure(9)
% plot(theta_plot,DragX,'k','LineWidth',1.5)
% hold on
% set(gca,'Fontname','Times New Roman','FontSize',14);
% xlim([0 pi])
% ylim([-1 6])
% xlabel('{\it\theta_{W}} [rad]','Fontname','Times New Roman','FontSize',14);
% ylabel('{\itF} [N]','Fontname','Times New Roman','FontSize',14);
% legend('F_{x}')
% 