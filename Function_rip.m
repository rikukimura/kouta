function [ Vdot ] = Functionrip(t,V,Mass,TreadWidth,seg_length,...
 omega,num_seg,scaleFactor,g,w_x,Rh,radii,w_c,L,sand_d,Cd,count)
% This is the function is used by ode45 function
index = zeros(num_seg,7);
% Local number, x,z,vx,vz,Beta,gamma
index(:,1) = [1:num_seg];
jr = 1;
if t < 4.752
    xS = V(3);
    yS = 56;
    elseif(4.752 < t) && (t < 9 )
     xS = V(3) + sqrt(200)/(9-4.752) * (t-4.752);
     yS = 56   + sqrt(200)/(9-4.752) * (t-4.752);
    else
     xS = V(3) + 20*cos(deg2rad(10)*(18-t));
     yS = 56   + 20*sin(deg2rad(10)*(18-t));
end
    theta_w = (- omega) * t ;
    
    xH = (Rh * cos(theta_w) + V(3));
    yH = (Rh * sin(theta_w) + w_c(2));
    xW = (radii * cos(theta_w) + V(3));%used for plot
    yW = (radii * sin(theta_w) + w_c(2));
    theta_pi =  atan2(yH - yS,xH - xS);
    if  theta_pi >0
        theta_pi = deg2rad(-180);
    end
    xP = xS + L*cos(theta_pi);
    yP = yS + L*sin(theta_pi); %パドル位置(使わない)

    
for jr = 1:num_seg
    
    index(jr,2) = xS + (L*cos(theta_pi)/num_seg) *jr  ; % New X position
    
    index(jr,3) = yS + (L*sin(theta_pi)/num_seg) *jr  ; % New Z position
    
    index(jr,4) = (V(1) + (omega * (index(jr,3) - V(4))));% New velocity in x-dir
        
    index(jr,5) =   (-omega * (index(jr,2) - V(3)));% New velocity in z-dir
    
    index(jr,6) = -theta_pi;
    
        if  index(jr,4) < 0
            index(jr,7) = atan(index(jr,5)./index(jr,4));
        elseif index(jr,4) >= 0
            index(jr,7) = atan(index(jr,5)./index(jr,4)) +pi;
        end
end



A00 = 0.206 ;
A10 = 0.169 ;
B11 = 0.212 ;
B01 = 0.358 ;
Bm11 = 0.055;
C11 = -0.124;
C01 = 0.253 ;
Cm11 = 0.007;
D10 = 0.088 ;

jr = 1;
ForceX = zeros(1,num_seg);
ForceZ = zeros(1,num_seg);
torque = zeros(1,num_seg);

index(100,3);
for jr =1:num_seg
    B = index(jr,6);
    G = index(jr,7);
    
    alphaX = scaleFactor * (Cm11 * cos(-2 * B + G) + ...
    C01 * cos(G) + C11 * cos(2 * B + G) + ...
    D10 * sin(2 * B));
    alphaZ = scaleFactor * (A10 * cos(2 * B) + A00 + ...
    Bm11 * sin((-2 * B) + G) + B01 * sin(G) + ...
    B11 * sin((2 * B) + G));

if index(jr,3) <= 0;
    ForceX(jr) = alphaX * seg_length * ...
    TreadWidth * -index(jr,3) * 0.001;
%     ForceX(jr) = alphaX * seg_length * ...
%     TreadWidth * -index(jr,3) ;
    ForceZ(jr) = alphaZ * seg_length * ...
    TreadWidth * -index(jr,3) * 0.001;

    Dragforce(jr) = 0.5 * (index(jr,4))^2 * sand_d * seg_length * TreadWidth * Cd  ;
    
else
    ForceX(jr) = 0;
    ForceZ(jr) = 0;
    Dragforce(jr) = 0;
end
posvec = [index(jr,2) - V(3);index(jr,3) - V(4);0];
forcevec = [ForceX(jr);ForceZ(jr);0];
torquecross = cross(posvec,forcevec);
torque(jr) = torquecross(3);

end

max(ForceX);
Torquetotal = sum(torque);
ForceXTot = sum(ForceX);
ForceZTot = sum(ForceZ);
dbforce   = sum(Dragforce);
Vdot = zeros(4,1);
aaa = (ForceXTot - dbforce) / Mass;
if index(num_seg,3)<=0
    Vdot(1) = ((ForceXTot - dbforce) / Mass)*1000;%mm/s^2
    % Accel in x-direction
    Vdot(2) = 0;
    % Accel in z-direction 
    Vdot(3) = V(1);% Velocity in x-dir
    Vdot(4) = V(2);% Velocity in z-dir
    Vdot(5) = omega * Torquetotal;
else
    Vdot(1) = 0;
    % Accel in x-direction
    Vdot(2) = 0;
    % Accel in z-direction 
    Vdot(3) = 0;% Velocity in x-dir
    Vdot(4) = 0;% Velocity in z-dir
    Vdot(5) = omega * Torquetotal;
% Derivitive of Energy dissipated
end