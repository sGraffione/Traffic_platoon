
% clear all;
% Metanet Network Data

%%%%%%%%%%%%%%%%%%% Parametri del modello

M=11; %set of freeway-links
O=1;  %set of origin-links
R=5;  %set of on-ramp links
D=2;  %set of destinations
N=9;  %set of nodes

K=1080;%%900; % Orizzonte temporale di 3 ore
T=1/3600*10; % Lunghezza intervallo di discretizzazione 10 s


Delta_M=zeros(M,1);
for m=1:M
   Delta_M(m,1)=700/1000; % Lunghezza delle sezioni per ciascun link (800 m)
end


chi_1=40;

delta_on_1=0.01;

nu_1=28;

tau_1=0.02;

l1=1;

m1=3;

rho_max=600;

rho_cr=150;


vf_1=100;%120

r_1_cap=1800;
r_1_min=0;

Q_max=6000;

%%%%%%% Freeway links %%%%%%%

% vettore colonna ogni elemento indica le destinazioni raggiungibili da ciascun link
J_m = zeros(M,1); 

J_m(1,1) = 2; % numero di destinazioni raggiungibili da M=1
J_m(2,1) = 2;  % numero di destinazioni raggiungibili da M=2
J_m(3,1) = 1;  % numero di destinazioni raggiungibili da M=3
J_m(4,1) = 1;  % numero di destinazioni raggiungibili da M=4
J_m(5,1) = 1;  % numero di destinazioni raggiungibili da M=5
J_m(6,1) = 2;  % numero di destinazioni raggiungibili da M=6
J_m(7,1) = 2;  % numero di destinazioni raggiungibili da M=7
J_m(8,1) = 1;  % numero di destinazioni raggiungibili da M=8
J_m(9,1) = 1;  % numero di destinazioni raggiungibili da M=9
J_m(10,1) = 1; % numero di destinazioni raggiungibili da M=10
J_m(11,1) = 1; % numero di destinazioni raggiungibili da M=11



% vettore colonna ogni elemento indica il numero di sezioni che appartengono a ciascun link
N_m = zeros(M,1);  

N_m(1,1) = 4;  % numero di sezioni per M=1
N_m(2,1) = 2;  % numero di sezioni per M=2
N_m(3,1) = 3;  % numero di sezioni per M=3
N_m(4,1) = 5;  % numero di sezioni per M=4
N_m(5,1) = 4;  % numero di sezioni per M=5
N_m(6,1) = 3;  % numero di sezioni per M=6
N_m(7,1) = 4;  % numero di sezioni per M=7
N_m(8,1) = 3;  % numero di sezioni per M=8
N_m(9,1) = 4;  % numero di sezioni per M=9
N_m(10,1) = 4;  % numero di sezioni per M=10
N_m(11,1) = 7;  % numero di sezioni per M=11


% initial condition

% initial density

rho_m1_initial = zeros(N_m(1,1),1); 
for i=1:N_m(1,1)
   rho_m1_initial(i,1) = 64;
end

rho_m2_initial = zeros(N_m(2,1),1); 
for i=1:N_m(2,1)
   rho_m2_initial(i,1) = 64;
end

rho_m3_initial = zeros(N_m(3,1),1);
for i=1:N_m(3,1)
   rho_m3_initial(i,1) = 45;
end

rho_m4_initial = zeros(N_m(4,1),1); 
for i=1:N_m(4,1)
   
   rho_m4_initial(i,1) = 55;

end
rho_m5_initial = zeros(N_m(5,1),1);
for i=1:N_m(5,1)
   rho_m5_initial(i,1) = 64;
end

rho_m6_initial = zeros(N_m(6,1),1); 
for i=1:N_m(6,1)
   rho_m6_initial(i,1) = 25;
end

rho_m7_initial = zeros(N_m(7,1),1); 
for i=1:N_m(7,1)
   rho_m7_initial(i,1) = 45;
end

rho_m8_initial = zeros(N_m(8,1),1); 
for i=1:N_m(8,1)
   rho_m8_initial(i,1) = 15;
end

rho_m9_initial = zeros(N_m(9,1),1); 
for i=1:N_m(9,1)
   rho_m9_initial(i,1) = 25;
end

rho_m10_initial = zeros(N_m(10,1),1); 
for i=1:N_m(10,1)
   rho_m10_initial(i,1) = 64;
end

rho_m11_initial = zeros(N_m(11,1),1); 
for i=1:N_m(11,1)
   rho_m11_initial(i,1) = 40;
end


rho_finD1 = 15;

rho_finD2 = 15;


% initial mean speed

v_m1_initial = zeros(N_m(1,1),1); 
for i=1:N_m(1,1)
   v_m1_initial(i,1) = vf_1*(1-(( rho_m1_initial(i,1) ) / rho_max )^l1)^m1;
end

v_m2_initial = zeros(N_m(2,1),1); 
for i=1:N_m(2,1)
   v_m2_initial(i,1) = vf_1*(1-(( rho_m2_initial(i,1) ) / rho_max )^l1)^m1;
end

v_m3_initial = zeros(N_m(3,1),1);
for i=1:N_m(3,1)
   v_m3_initial(i,1) = vf_1*(1-(( rho_m3_initial(i,1)  ) / rho_max )^l1)^m1;
end

v_m4_initial = zeros(N_m(4,1),1); 
for i=1:N_m(4,1)
   v_m4_initial(i,1) = vf_1*(1-(( rho_m4_initial(i,1) ) / rho_max )^l1)^m1;
end

v_m5_initial = zeros(N_m(5,1),1);
for i=1:N_m(5,1)
   v_m5_initial(i,1) = vf_1*(1-(( rho_m5_initial(i,1)  ) / rho_max )^l1)^m1;
end

v_m6_initial = zeros(N_m(6,1),1); 
for i=1:N_m(6,1)
   v_m6_initial(i,1) = vf_1*(1-(( rho_m6_initial(i,1) ) / rho_max )^l1)^m1;
end

v_m7_initial = zeros(N_m(7,1),1); 
for i=1:N_m(7,1)
   v_m7_initial(i,1) = vf_1*(1-(( rho_m7_initial(i,1) ) / rho_max )^l1)^m1;
end

v_m8_initial = zeros(N_m(8,1),1); 
for i=1:N_m(8,1)
   v_m8_initial(i,1) = vf_1*(1-(( rho_m8_initial(i,1)  ) / rho_max )^l1)^m1;
end

v_m9_initial = zeros(N_m(9,1),1); 
for i=1:N_m(9,1)
   v_m9_initial(i,1) = vf_1*(1-(( rho_m9_initial(i,1) ) / rho_max )^l1)^m1;
end

v_m10_initial = zeros(N_m(10,1),1); 
for i=1:N_m(10,1)
   v_m10_initial(i,1) = vf_1*(1-(( rho_m10_initial(i,1) ) / rho_max )^l1)^m1;
end

v_m11_initial = zeros(N_m(11,1),1); 
for i=1:N_m(11,1)
   v_m11_initial(i,1) = vf_1*(1-(( rho_m11_initial(i,1) ) / rho_max )^l1)^m1;
end



% initial flow

q_m1_initial = zeros(N_m(1,1),1); 
for i=1:N_m(1,1)
   q_m1_initial(i,1) = rho_m1_initial(i,1) * v_m1_initial(i,1);
end

q_m2_initial = zeros(N_m(2,1),1); 
for i=1:N_m(2,1)
   q_m2_initial(i,1) = rho_m2_initial(i,1) * v_m2_initial(i,1);
end

q_m3_initial = zeros(N_m(3,1),1);
for i=1:N_m(3,1)
   q_m3_initial(i,1) = rho_m3_initial(i,1) * v_m3_initial(i,1);
end

q_m4_initial = zeros(N_m(4,1),1); 
for i=1:N_m(4,1)
   q_m4_initial(i,1) = rho_m4_initial(i,1) * v_m4_initial(i,1);
end

q_m5_initial = zeros(N_m(5,1),1);
for i=1:N_m(5,1)
   q_m5_initial(i,1) = rho_m5_initial(i,1) * v_m5_initial(i,1);
end

q_m6_initial = zeros(N_m(6,1),1); 
for i=1:N_m(6,1)
   q_m6_initial(i,1) = rho_m6_initial(i,1) * v_m6_initial(i,1);
end

q_m7_initial = zeros(N_m(7,1),1); 
for i=1:N_m(7,1)
   q_m7_initial(i,1) = rho_m7_initial(i,1) * v_m7_initial(i,1);
end

q_m8_initial = zeros(N_m(8,1),1); 
for i=1:N_m(8,1)
   q_m8_initial(i,1) = rho_m8_initial(i,1) * v_m8_initial(i,1);
end

q_m9_initial = zeros(N_m(9,1),1); 
for i=1:N_m(9,1)
   q_m9_initial(i,1) = rho_m9_initial(i,1) * v_m9_initial(i,1);
end

q_m10_initial = zeros(N_m(10,1),1); 
for i=1:N_m(10,1)
   q_m10_initial(i,1) = rho_m10_initial(i,1) * v_m10_initial(i,1);
end

q_m11_initial = zeros(N_m(11,1),1); 
for i=1:N_m(11,1)
   q_m11_initial(i,1) = rho_m11_initial(i,1) * v_m11_initial(i,1);
end





%%%%%%% Origin links %%%%%%%

% set of possible destination reachable from O
J_o = zeros(O,1);
J_o(1,1) = 2; % destinations reachable from origin O1

% origin demand o1

rho_o1 = zeros(1,K+1);

for k=1:K+1
if (k>=1 && k<(630)) 
    rho_o1(1,k)=70;
end
if (k>=630 && k<=K)
    rho_o1(1,k)=50;    
end

end

v_o1 = zeros(1,K+1);

for k=1:K+1
v_o1(1,k) = vf_1 * ( 1 - ( ( rho_o1(1,k)  ) / rho_max ) ^ l1 )^ m1;
end

demand_o1 = zeros(1,(K+1)); 
for k=1:K+1
   demand_o1(1,k) = rho_o1(1,k) * v_o1(1,k);
end

% rate of demand with destination in J_o
theta_o1 = zeros(J_o(1,1),K+1); % class 1 origin 1
for k=1:K+1
    theta_o1(1,k) = 0.65; %destination D1
    theta_o1(2,k) = 0.35; %destination D2
end


%%%%%%% On-ramp links %%%%%%%

% on-ramp demand 
demand_o3 = zeros(1,(K+1)); % demand class 1 for on-ramp 1


% demand for on-ramp o3
for k=1:K 
if (k>=1 && k<90)
   demand_o3(1,k)=620 +(1100-620)/90*k;     
end
if (k>=90 && k<(630)) 
   demand_o3(1,k)=1100;
end
if (k>=630 && k<360*2)
   demand_o3(1,k)=1150-(1100-630)/(90)*(k-630);    
end
if (k>=360*2 && k<=K)
   demand_o3(1,k)=630;    
end    

end



%  demand for on-ramp o5

demand_o5 = zeros(1,(K+1)); 

for k=1:K
   demand_o5(1,k) = 5000; 
end




% demand for on-ramp o2

demand_o2 = zeros(1,(K+1)); % demand class 1 for on-ramp 3

for k=1:K  
   if (k>=1 && k<90)
     demand_o2(1,k)=530 + (850-530)/90*k;     
   end
   if (k>=90 && k<(630))
     demand_o2(1,k)=850;
   end
   if (k>=630 && k<360*2)
     demand_o2(1,k)=850-(850-410)/(90)*(k-630);    
   end
   if (k>=360*2 && k<=K)
     demand_o2(1,k)=410;    
   end
end



% demand for on-ramp o4

demand_o4 = zeros(1,(K+1));

for k=1:K  
    if (k>=1 && k<300)
     demand_o4(1,k)=300;     
   end
   if (k>=300 && k<390)
     demand_o4(1,k)=300 + (850-300)/90*(k-300);     
   end
   if (k>=390 && k<(630))
     demand_o4(1,k)=850;
   end
   if (k>=630 && k<360*2)
     demand_o4(1,k)=850-(850-430)/(90)*(k-630);    
   end
   if (k>=360*2 && k<=K)
     demand_o4(1,k)=430;    
   end
end


% demand for on-ramp o6

demand_o6 = zeros(1,(K+1)); 

for k=1:K  
   if (k>=1 && k<90)
     demand_o6(1,k)=830 + (1255-830)/90*k;     
   end
   if (k>=90 && k<(630)) 
     demand_o6(1,k)=1255;
   end
   if (k>=630 && k<360*2)
     demand_o6(1,k)=1255-(1255-510)/(90)*(k-630);    
   end
   if (k>=360*2 && k<=K)
     demand_o6(1,k)=510;    
   end
end


% set of possible destination reachable from oringin ramp
J_r = zeros(R,1);
J_r(1,1) = 1; % destinations reachable from on-ramp o3
J_r(2,1) = 2; % destinations reachable from on-ramp o5
J_r(3,1) = 2; % destinations reachable from on-ramp o3
J_r(4,1) = 1; % destinations reachable from on-ramp o4
J_r(5,1) = 1; % destinations reachable from on-ramp o5

% rate of demand with destination in J_r
theta_o3 = zeros(J_r(1,1),K+1); 
for k=1:K+1
    theta_o3(1,k) = 1; %destination D1
end

% rate of demand with destination in J_r
theta_o5 = zeros(J_r(2,1),K+1); 
for k=1:K+1
    theta_o5(1,k) = 0; %destination D1
    theta_o5(2,k) = 1; %destination D2
end

% rate of demand with destination in J_r
theta_o2 = zeros(J_r(3,1),K+1); 
for k=1:K+1
    theta_o2(1,k) = 0.45; %destination D1
    theta_o2(2,k) = 0.55; %destination D1
end

% rate of demand with destination in J_r
theta_o4 = zeros(J_r(4,1),K+1); 
for k=1:K+1
    theta_o4(1,k) = 1; %destination D1
end

% rate of demand with destination in J_r
theta_o6 = zeros(J_r(5,1),K+1); 
for k=1:K+1
    theta_o6(1,k) = 1; %destination D2
end

%%%%%%% Nodes %%%%%%%

% set of possible destination reachable from N
J_n = zeros(N,1);

J_n(1,1) = 2; % destinations reachable from node N1
J_n(2,1) = 2; % destinations reachable from node N2
J_n(3,1) = 2; % destinations reachable from node N3
J_n(4,1) = 1; % destinations reachable from node N4
J_n(5,1) = 1; % destinations reachable from node N5
J_n(6,1) = 1; % destinations reachable from node N6
J_n(7,1) = 2; % destinations reachable from node N7
J_n(8,1) = 2; % destinations reachable from node N8
J_n(9,1) = 1; % destinations reachable from node N9


% %% Parametri modelli di emissione
% 
% %auto
% 
% u0_1= 2.5719e-4;
% u1_1=0.0011;
% u2_1=0.0053;
% u3_1=0.0069;
% u4_1=0.0028;
% u5_1=7.2806e-4;
% u6_1=0.0254;
% u7_1=0.0013;
% u8_1=0.0035;
% u9_1=0.0019;
% 
% 
% 
% %% velocità in ingresso sulla rampa
% 
% v_on_1=zeros(R,K+1);
% 
% for i=1:R;
%     for k=1:K+1
%     v_on_1(i,k)=30;
%     end
% end
% 
% %% velocità al minimo sulla rampa
% 
% v_idl_1=zeros(R,K+1);
% 
% for i=1:R;
%     for k=1:K+1
%     v_idl_1(i,k)=5;
%     end
% end
% 
% %% conversione km/h^2  a me/s^2
% w_par=0.014;
% 
% conv_fact=1000/3600^2;
% 
% Tsec=10; %delta t in secondi

%%%%%%
