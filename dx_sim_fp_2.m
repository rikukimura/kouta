clf;
close;
clear all%ワークスペースの数値を初期化

% L =0.096;%[mm]
L =76;%[m]
Rh =50;%[mm]ヒンジ半径
theta_w = 0; %車輪モータ初期値
i=0;
n = 1000;%円を描くときの分割数n
m=0;
k=0;
p=0;
theta_w =zeros(n,1);
rS =20 ;
xS=zeros(n,1);%シャフト位置
yS=zeros(n,1);
rec = zeros(n,3);
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

%%
% theta = (linspace(0,- pi,n)).';%unit rad
% x = rS*cos(theta) ;  
% y = rS*sin(theta) ;

x = w_c(1); 
y = w_c(2);
xS(i+1:i+n,1) = x.' ;
yS(i+1:i+n,1) = y.' ;


%% パドルの手先位置の移動速度Δvx,vyの計算
omega = 10 ;%deg/s
duration = 180 / omega ; %test time
dt = duration / (n-1)      ;
q = 1;
%%
k =1;
for theta_w = 0 :deg2rad(-180/(n-1)) : deg2rad(-180)
xH(k) = Rh * cos(theta_w) + w_c(1);
yH(k) = Rh * sin(theta_w) + w_c(2);
k = k+1;
end
yH = yH.';
xH = xH.';
 theta_pi = atan2(yH - yS,xH - xS);
if theta_pi(n) >0
    theta_pi(n) = -180;
end
 xP = xS + L*cos(theta_pi);
 yP = yS + L*sin(theta_pi); %パドル位置
 
 num_seg = 250 ;
 seg_length = L / num_seg ;%length of one segment
 i =1 ;
 
 for i = 1:n 
    for q = 1:num_seg
      seg_y(i,q) = yS(i) + (L*sin(theta_pi(i))/num_seg) * q  ;
      seg_x(i,q) = xS(i) + (L*cos(theta_pi(i))/num_seg) * q  ;
    end
 end
 
 

 i = 1;
dx(i,1:num_seg) = 0;
dy(i,1:num_seg) = 0;

d_vx = diff(seg_x);
d_vy = diff(seg_y);
%% B and G

 for i = 1:n-1
     for q = 1 : num_seg
         
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
        
%          Dragforce(i,q) = 0.5 * (d_vx(i,q))^2 * sand_d * seg_length *TreadWidth* Cd *0.0001 ;
         if Z(i,q) < 0
            ForceX(i,q) = sigma_x(i,q) *  TreadWidth * seg_length * 0.01  ;
            ForceZ(i,q) = sigma_z(i,q) * TreadWidth * seg_length * 0.01  ;
         else
             ForceX(i,q) = 0 ;
             ForceZ(i,q) = 0 ;
         end
         
      end
    ForceX_sum(i) = sum(ForceX(i,1:end));
    ForceZ_sum(i) = sum(ForceZ(i,1:end));
%     DragX(i)      = sum(Dragforce(i,1:end));
    seg_sepX(i) = xP(i)/num_seg;
    seg_sepY(i) = yP(i)/num_seg;
    
    a(i) = ForceX_sum(i) / Mass ;%[m/s^2]
    if i == 1
        v_x(i) =  0;
        mov_x(i) = 0;
    else
        v_x(i) = v_x(i-1)+a(i)*dt;
        mov_x(i) = v_x(i-1)*dt + 1/2 *a(i)*dt*dt;
    end
 end
 
 %% plot area
%  
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
% figure(3)
% plot(xP,yP);
% daspect([1 1 1]);
 theta_plot = linspace(0,pi,n-1);
%  figure(4)
 plot(theta_plot,ForceX_sum,'k','LineWidth',1.5) 
 hold on
 plot(theta_plot,ForceZ_sum,'r','LineWidth',1.5)
 xlim([0 pi])
 ylim([-1 6])
 xlabel('{\it\theta_{W}} [rad]','Fontname','Times New Roman','FontSize',14);
 ylabel('{\itF} [N]','Fontname','Times New Roman','FontSize',14);
 legend('F_{x}','F_{y}')
%  saveas(gcf,'sim-fp.svg')
