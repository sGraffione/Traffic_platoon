% j2=K_finB;
v_traffic_section_leader1 = zeros (j1-1,1);
v_traffic_section_leader2 = zeros (j2-j1-1,1);
v_traffic_section_leader3 = zeros (j3-j2,1);
v_traffic_section_follower1 = zeros (A+1,1);
v_traffic_section_follower2 = zeros (B+1,1);

v_traffic11 = zeros(N_path1,K_finA); %speed of traffic in platoon path

v_traffic11(1:N_m(1,1),:) = v_m1(2:N_m(1,1)+1,1+K_leader1:K_finA+K_leader1);
v_traffic11(N_m(1,1)+ 1 : N_m(1,1)+ N_m(2,1) +1 ,:) = v_m2(:,1+K_leader1:K_finA+K_leader1);
v_traffic11(N_m(1,1)+ N_m(2,1)+1 : N_m(1,1)+ N_m(2,1)+ N_m(3,1) +1 , :) = v_m3(:,1+K_leader1:K_finA+K_leader1);
v_traffic11(N_m(1,1)+ N_m(2,1)+ N_m(3,1) +1 : N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1) +1 , :) = v_m4(:,1+K_leader1:K_finA+K_leader1);
v_traffic11(N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1)+ 1 : N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1) + N_m(5,1) +1 , :) = v_m5(:,1+K_leader1:K_finA+K_leader1);
v_traffic11(N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1)+ N_m(5,1)+ 1 : N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1) + N_m(5,1)+ N_m(10,1) +1 , :) = v_m10(:,1+K_leader1:K_finA+K_leader1);
v_traffic11(N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1) + N_m(5,1)+ N_m(10,1) +2 : N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1) + N_m(5,1)+ N_m(10,1) +3 , :) = v_m10(N_m(10,1) : N_m(10,1)+1,1+K_leader1:K_finA+K_leader1);
% 
% 
v_traffic22 = zeros(N_path2,K_finB); %speed of traffic in platoon path

v_traffic22(1:N_m(1,1),:) = v_m1(2:N_m(1,1)+1,1+K_leader2:K_finB+K_leader2);
v_traffic22(N_m(1,1)+ 1 : N_m(1,1)+ N_m(2,1) +1 ,:) = v_m2(:,1+K_leader2:K_finB+K_leader2);
v_traffic22(N_m(1,1)+ N_m(2,1)+1 : N_m(1,1)+ N_m(2,1)+ N_m(3,1) +1 , :) = v_m3(:,1+K_leader2:K_finB+K_leader2);
v_traffic22(N_m(1,1)+ N_m(2,1)+ N_m(3,1) +1 : N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1) +1 , :) = v_m4(:,1+K_leader2:K_finB+K_leader2);
v_traffic22(N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1)+ 1 : N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1) + N_m(5,1) +1 , :) = v_m5(:,1+K_leader2:K_finB+K_leader2);
v_traffic22(N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1)+ N_m(5,1)+ 1 : N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1) + N_m(5,1)+ N_m(10,1) +1 , :) = v_m10(:,1+K_leader2:K_finB+K_leader2);
v_traffic22(N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1) + N_m(5,1)+ N_m(10,1) +2 : N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1) + N_m(5,1)+ N_m(10,1) +5 , :) = v_m10(N_m(10,1)-2 : N_m(10,1)+1,1+K_leader2:K_finB+K_leader2);
% 
% 
v_traffic33 = zeros(N_path3,K_finC); %speed of traffic in platoon path

v_traffic33(1:N_m(1,1),:) = v_m1(2:N_m(1,1)+1,1+K_leader3:K_finC+K_leader3);
v_traffic33(N_m(1,1)+ 1 : N_m(1,1)+ N_m(2,1) +1 ,:) = v_m2(:,1+K_leader3:K_finC+K_leader3);
v_traffic33(N_m(1,1)+ N_m(2,1)+1 : N_m(1,1)+ N_m(2,1)+ N_m(3,1) +1 , :) = v_m3(:,1+K_leader3:K_finC+K_leader3);
v_traffic33(N_m(1,1)+ N_m(2,1)+ N_m(3,1) +1 : N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1) +1 , :) = v_m4(:,1+K_leader3:K_finC+K_leader3);
v_traffic33(N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1)+ 1 : N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1) + N_m(5,1) +1 , :) = v_m5(:,1+K_leader3:K_finC+K_leader3);
v_traffic33(N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1)+ N_m(5,1)+ 1 : N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1) + N_m(5,1)+ N_m(10,1) +1 , :) = v_m10(:,1+K_leader3:K_finC+K_leader3);
v_traffic33(N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1) + N_m(5,1)+ N_m(10,1) +2: N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1) + N_m(5,1)+ N_m(10,1) +5 , :) = v_m4(N_m(4,1)-2:N_m(4,1)+1,1+K_leader3:K_finC+K_leader3);
v_traffic33(N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1) + N_m(5,1)+ N_m(10,1) +6: N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1) + N_m(5,1)+ N_m(10,1) +9 , :) = v_m5(N_m(5,1)-2:N_m(5,1)+1,1+K_leader3:K_finC+K_leader3);
v_traffic33(N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1) + N_m(5,1)+ N_m(10,1) +10: N_m(1,1)+ N_m(2,1)+ N_m(3,1)+ N_m(4,1) + N_m(5,1)+ N_m(10,1) +11 , :) = v_m10(N_m(10,1):N_m(10,1)+1,1+K_leader3:K_finC+K_leader3);


v_traffic_follower11 = zeros(N_path_follower1,A); %speed of traffic in platoon path

v_traffic_follower11(1:N_m(2,1),:) = v_m2(2:N_m(2,1)+1,K_follower+1:A+K_follower);
v_traffic_follower11(N_m(2,1)+ 1 : N_m(2,1)+ N_m(6,1) +1 ,:) = v_m6(:,K_follower+1:A+K_follower);
v_traffic_follower11(N_m(2,1)+ N_m(6,1)+1 : N_m(2,1)+ N_m(6,1)+ N_m(7,1) +1 , :) = v_m7(:,K_follower+1:A+K_follower);
v_traffic_follower11(N_m(2,1)+ N_m(6,1)+ N_m(7,1) +1 : N_m(2,1)+ N_m(6,1)+ N_m(7,1)+ N_m(8,1) +1 , :) = v_m8(:,K_follower+1:A+ K_follower);
v_traffic_follower11(N_m(2,1)+ N_m(6,1)+ N_m(7,1)+ N_m(8,1) +2 : N_m(2,1)+ N_m(6,1)+ N_m(7,1)+ N_m(8,1) +4 , :) = v_m2(N_m(2,1)-1:N_m(2,1)+1,K_follower+1:A+K_follower);
v_traffic_follower11(N_m(2,1)+ N_m(6,1)+ N_m(7,1)+ N_m(8,1) +5 : N_m(2,1)+ N_m(6,1)+ N_m(7,1)+ N_m(8,1) +7 , :) = v_m6(N_m(6,1)-2:N_m(6,1),K_follower+1:A+K_follower);
v_traffic_follower11(N_m(2,1)+ N_m(6,1)+ N_m(7,1)+ N_m(8,1) +8 : N_m(2,1)+ N_m(6,1)+ N_m(7,1)+ N_m(8,1) +10 , :) = v_m7(N_m(7,1)-3:N_m(7,1)-1,K_follower+1:A+K_follower);
v_traffic_follower11(N_m(2,1)+ N_m(6,1)+ N_m(7,1)+ N_m(8,1) +11 : N_m(2,1)+ N_m(6,1)+ N_m(7,1)+ N_m(8,1) +12 , :) = v_m8(N_m(8,1):N_m(8,1)+1,K_follower+1:A+K_follower);
% 
v_traffic_follower22 = zeros(N_path_follower2,B); %speed of traffic in platoon path

v_traffic_follower22(1:N_m(2,1),:) = v_m2(2:N_m(2,1)+1,1:B);
v_traffic_follower22(N_m(2,1)+ 1 : N_m(2,1)+ N_m(6,1) +1 ,:) = v_m6(:,1:B);
v_traffic_follower22(N_m(2,1)+ N_m(6,1)+1 : N_m(2,1)+ N_m(6,1)+ N_m(7,1) +1 , :) = v_m7(:,1:B);
v_traffic_follower22(N_m(2,1)+ N_m(6,1)+ N_m(7,1) +1 : N_m(2,1)+ N_m(6,1)+ N_m(7,1)+ N_m(8,1) +1 , :) = v_m8(:,1:B);
v_traffic_follower22(N_m(2,1)+ N_m(6,1)+ N_m(7,1)+ N_m(8,1) +2 : N_m(2,1)+ N_m(6,1)+ N_m(7,1)+ N_m(8,1) +4 , :) = v_m2(N_m(2,1)-1:N_m(2,1)+1,1:B);
v_traffic_follower22(N_m(2,1)+ N_m(6,1)+ N_m(7,1)+ N_m(8,1) +5 : N_m(2,1)+ N_m(6,1)+ N_m(7,1)+ N_m(8,1) +7 , :) = v_m6(N_m(6,1)-2:N_m(6,1),1:B);
v_traffic_follower22(N_m(2,1)+ N_m(6,1)+ N_m(7,1)+ N_m(8,1) +8 : N_m(2,1)+ N_m(6,1)+ N_m(7,1)+ N_m(8,1) +10 , :) = v_m11(N_m(11,1)-6:N_m(11,1)-4,1:B);
v_traffic_follower22(N_m(2,1)+ N_m(6,1)+ N_m(7,1)+ N_m(8,1) +11 : N_m(2,1)+ N_m(6,1)+ N_m(7,1)+ N_m(8,1) +13 , :) = v_m11(N_m(11,1)-3:N_m(11,1)-1,1:B);
v_traffic_follower22(N_m(2,1)+ N_m(6,1)+ N_m(7,1)+ N_m(8,1) +14 : N_m(2,1)+ N_m(6,1)+ N_m(7,1)+ N_m(8,1) +16 , :) = v_m8(N_m(8,1)-1:N_m(8,1)+1,1:B);

for i=1:(j1-1)

v_traffic_section_leader1(i,1) = v_traffic11(cell_L(1,i),i);

end

for i=1:(j2-j1)

v_traffic_section_leader2(i,1) = v_traffic22(cell_L(1,i+j1-1),i);

end
% % 
for i=1:j3-j2

v_traffic_section_leader3(i,1) = v_traffic33(cell_L(1,i+j2-1),i);

end


for i=1:j1-1

v_traffic_section_follower1(i,1) = v_traffic_follower11(cell_F1(1,i),i);

end


for i=1:j2-j1-1

v_traffic_section_follower2(i,1) = v_traffic_follower22(cell_F2(1,i+j1),i);

end





i=1:j1-1;
figure;
plot(i,vLeader(1,1:j1-1),i,v_traffic_section_leader1(1:j1-1,1));
% title 'Speed of platoon L vs the mean speed of traffic in the same section';
xlabel ('Time steps [k]');
ylabel('Speed [km/h]');
legend('speed of the leader in the first part of the trip','mean speed of the section where the leader is');
% saveas(gcf,'leaderspeed500',"m");


i=1:j2-j1;
figure;
plot(i,vLeader(1,j1:j2-1),i,v_traffic_section_leader2(i,1));
% title 'Speed of platoon L vs the mean speed of traffic in the same section';
xlabel ('Time steps [k]');
ylabel('Speed [km/h]');
legend('speed of the leader in the second part of the trip','mean speed of the section where the leader is');
% saveas(gcf,'leaderspeed500',"m");
% 
i=1:j3-j2;
figure;
plot(i,vLeader(1,j2:j3-1),i,v_traffic_section_leader3(i,1));
% title 'Speed of platoon L vs the mean speed of traffic in the same section';
xlabel ('Time steps [k]');
ylabel('Speed [km/h]');
legend('speed of the leader in the third part of the trip','mean speed of the section where the leader is');
% % saveas(gcf,'leaderspeed500',"m");

i=1:j1;
figure;
plot(i,vFollower1(1,1:j1),i,v_traffic_section_follower1(i,1));
% title 'Speed of platoon F1 vs the mean speed of traffic in the same section';
xlabel ('Time steps [k]');
ylabel('Speed [km/h]');
legend('speed of the first follower','mean speed of the section where the first follower is');

i=1:j2-j1;
figure;
plot(i,vFollower2(1,j1+1:j2),i,v_traffic_section_follower2(i,1));
% title 'Speed of platoon F2 vs the mean speed of traffic in the same section';
xlabel ('Time steps [k]');
ylabel('Speed [km/h]');
legend('speed of the second follower','mean speed of the section where the second follower is');





