clf;
close;
clear %ワークスペースの数値を初期化

L =76;%[m]
Rh =50;%[mm]ヒンジ半径
theta_w = 0; %車輪モータ初期値
n = 1000;%円を描くときの分割数n
num_seg = 100 ;
m=0;
k=0;
p=0;
rS =20 ;
theta_w =zeros(n,1);
% xS=zeros(n,1);%シャフト位置
% yS=zeros(n,1);
xP = zeros(n,1);
yP = zeros(n,1);
xS = zeros(n,1);
yS = zeros(n,1);
xH = zeros(n,1);
yH = zeros(n,1);
theta_pi = zeros(n,1);

seg_x = zeros(n,num_seg);
seg_y = zeros(n,num_seg);
G     = zeros(n,num_seg);
B     = zeros(n,num_seg);
z     = zeros(n,num_seg);
Z     = zeros(n,num_seg);

alphaX = zeros(n,num_seg);
alphaZ = zeros(n,num_seg);
sigma_x = zeros(n,num_seg);
sigma_z = zeros(n,num_seg);

ForceX = zeros(n,num_seg);
ForceZ = zeros(n,num_seg);

ForceX_sum = zeros(n,1);
ForceZ_sum = zeros(n,1);

w_x = zeros(n,1);
 
w_c = [0,56];%wheel center
Mass = 7.2; %[kg]
TreadWidth =63;%[mm]
scaleFactor = 1.1;
sand_d = 1480; % dencity of sand [kg/m^3]
% sand_d = sand_d * 10^(-9);
Cd     = 2 ;   % Drag coefficient
A00 = 0.206 ;
A10 = 0.169 ;
B11 = 0.212 ;
B01 = 0.358 ;
Bm11 = 0.055;
C11 = -0.124;
C01 = 0.253 ;
Cm11 = 0.007;
D10 = 0.088 ;

seg_length = L / num_seg ;%length of one segment
i =0;
dx(1,1:num_seg) = 0;
dy(1,1:num_seg) = 0;
d_vx(1,1:num_seg) = 0;
d_vy(1,1:num_seg) = 0;
theta_w(1) = 0;
%%
% theta = (linspace(0,- pi,n)).';%unit rad
% x = rS*cos(theta) ;  
% y = rS*sin(theta) ;

x = w_c(1); 
y = w_c(2);
w_x(1) =w_c(1);
% xS(i+1:i+n,1) = x.' ;
% yS(i+1:i+n,1) = y.' ;


%% パドルの手先位置の移動速度Δvx,vyの計算
omega = 10 ;%deg/s
duration = deg2rad(180) / omega ; %test time
dt = duration / n    ;
q = 1;
%%
k =1;
for i=1:n
    xS(i) = w_x(i);
    yS(i) = y;
    xH(i,1) = (Rh * cos(theta_w(i,1)) + w_x(i,1));
    yH(i) = (Rh * sin(theta_w(i,1)) + w_c(2));
    theta_w = theta_w + (- omega) * dt ;
    theta_pi(i,1) =  atan2(yH(i) - yS(i),xH(i) - xS(i));
    if  theta_pi(i,1) >0
        theta_pi(i,1) = deg2rad(-180);
    end
    xP(i) = xS(i) + L*cos(theta_pi(i));
    yP(i) = yS(i) + L*sin(theta_pi(i)); %パドル位置

  for q = 1:num_seg
      seg_y(i,q) = yS(i) + (L*sin(theta_pi(i))/num_seg) * q  ;
      seg_x(i,q) = xS(i) + (L*cos(theta_pi(i))/num_seg) * q  ;
      if i >1
          d_vx(i,q) = (seg_x(i,q) - seg_x(i-1,q))/dt;
          d_vy(i,q) = (seg_y(i,q) - seg_y(i-1,q))/dt;
      end

         B(i,q) = -theta_pi(i,1);
         if i == 1
           G(i,q) = atan(d_vy(i,q)./d_vx(i,q));
         else
           G(i,q) = atan(d_vy(i,q)./(d_vx(i,q)+ v_x(i-1)));  
         end
%% z
        z(i,q) = seg_y(i,q);

        Z(i,q) = z(i,q);
        Z(Z>0) = 0;        
     
         alphaX(i,q)=(Cm11 * cos(-2 * B(i,q) + G(i,q)) + C01 * cos(G(i,q)) + ...
                        C11 * cos(2 * B(i,q) + G(i,q)) + D10 * sin(2 * B(i,q)));
         alphaZ(i,q) = (A10 * cos(2 * B(i,q)) + A00 + Bm11 * sin((-2 * B(i,q)) + G(i,q)) + ...
                        B01 * sin(G(i,q)) + B11 * sin((2 * B(i,q)) + G(i,q)));
                    
         sigma_x(i,q) = alphaX(i,q) * -Z(i,q) *scaleFactor * 0.1 ;
         sigma_z(i,q) = alphaZ(i,q) * -Z(i,q) *scaleFactor * 0.1 ;
        
        
         if Z(i,q) < 0
             
            ForceX(i,q) = sigma_x(i,q) *  TreadWidth * seg_length * 0.01  ;
            ForceZ(i,q) = sigma_z(i,q) * TreadWidth * seg_length  * 0.01  ;
%             ForceX(i,q) = sigma_x(i,q) *  TreadWidth * seg_length *cos(B(i,q))* 0.01  ;
%             ForceZ(i,q) = sigma_z(i,q) * TreadWidth * seg_length *sin(B(i,q)) * 0.01  ;
%             Dragforce(i,q) = 0.5 * (d_vx(i,q))^2 * sand_d * seg_lengt]h *sin(B(i,q))* TreadWidth * Cd * 0.00001 ;
         else
             ForceX(i,q) = 0 ;
             ForceZ(i,q) = 0 ;
%              Dragforce(i,q) =0;
         end
         
%          ForceX(i,q) = sigma_x(i,q) *  TreadWidth * -z(i,q) * 0.01 / 2 ;
%          ForceZ(i,q) = sigma_z(i,q) * TreadWidth * -z(i,q) * 0.01  / 2;        
          

%           ForceX(i,q) = sigma_x(i,q) * seg_length * TreadWidth * -z(i,q) * 0.01 /2 ;
%           ForceZ(i,q) = sigma_z(i,q) * seg_length * TreadWidth * -z(i,q) * 0.01 /2;
          
%           ForceX(i,q) = alphaX(i,q) * (xP(i)/num_seg) * TreadWidth ;
%           ForceZ(i,q) = alphaZ(i,q) * (yP(i)/num_seg) * TreadWidth ;
   end
    ForceX_sum(i) = sum(ForceX(i,1:end));
    ForceZ_sum(i) = sum(ForceZ(i,1:end));
%     DragX(i)      = sum(Dragforce(i,1:end));
    Net_Fx(i) =  ForceX_sum(i);%-DragX(i);
    a(i) = Net_Fx(i) / Mass ;%[m/s^2]
    if i == 1
        v_x(i) =  0;
        mov_x(i) = 0;
        w_x(i+1) = w_x(i);%次の車輪の中心位置座標[x]
        
    else if a(i) ~= 0    
        v_x(i) = v_x(i-1)+a(i)*dt;
        mov_x(i) = v_x(i-1)*dt + 1/2 *a(i)*dt*dt;
        w_x(i+1) = mov_x(i) * 1000 + w_x(i); %change unit [m] to [mm]
        else
            v_x(i) = 0;
            mov_x(i) = mov_x(i-1);
            w_x(i+1) = w_x(i); 
        end
    end
end

%% Animation part( future work )
 %% plot area
% figure(1)
% plot(xS,yS,'k','LineWidth',1.5,'MarkerSize',100);
% xlim([-50,50]);
% ylim([0,100]);
% daspect([1 1 1]);
% 
% figure(2)
% plot(xH,yH)
% daspect([1 1 1]);
% 
figure(3)
plot(xP,yP);
daspect([1 1 1]);
theta_plot = linspace(0,pi,n);
 figure(4)
plot(theta_plot,ForceX_sum,'k','LineWidth',1.5) 
 hold on
plot(theta_plot,ForceZ_sum,'r','LineWidth',1.5)
xlim([0 pi])
ylim([-1 6])
xlabel('{\it\theta_{W}} [rad]','Fontname','Times New Roman','FontSize',14);
ylabel('{\itF} [N]','Fontname','Times New Roman','FontSize',14);
legend('F_{x}','F_{y}')
% %  saveas(gcf,'sim-fp.svg')
t = linspace(0,18,n-1);
% figure(5)
% plot(t,a,'LineWidth',1.5)
% xlabel('time[s]','Fontname','Times New Roman','FontSize',14);
% ylabel('{\ita{x}} [m/s^2]','Fontname','Times New Roman','FontSize',14);
% figure(6)
% plot(t,v_x,'LineWidth',1.5)
% xlabel('time[s]','Fontname','Times New Roman','FontSize',14);
% ylabel('{\itv_{x}} [m/s]','Fontname','Times New Roman','FontSize',14);
% figure(7)
% plot(t,mov_x,'LineWidth',1.5)
% xlabel('time[s]','Fontname','Times New Roman','FontSize',14);
% ylabel('{distance} [m]','Fontname','Times New Roman','FontSize',14);
