%% No Control Model


for k2=1:K


    %%% origin link model %%%

    % partial demand
    for j=1:J_o(1,1) % Destination reachable from O1
        partial_demand_o1(j,k2) = demand_o1(1,k2) * theta_o1(j,k2); % cars

    end

    % origin flow O1

    q_o1(1,k2) = min( [ demand_o1(1,k2) + l_o1(1,k2) / T ; Q_max ;...
        Q_max * ( ( rho_max - rho_m1(1,k2)) / (rho_max - rho_cr ) ) ]);

    for j=1:J_o(1,1)
        if ( partial_l_o1(j,k2) == 0 && l_o1(1,k2)==0 )
            gamma_o1(j,k2) = theta_o1(j,k2);
        else
            gamma_o1(j,k2) = partial_l_o1(j,k2) / l_o1(1,k2);
        end
    end


    %  partial origin queue O1
    for j=1:J_o(1,1)
        partial_l_o1(j,k2+1) = partial_l_o1(j,k2) + T * ( demand_o1(1,k2) * theta_o1(j,k2) - gamma_o1(j,k2) * q_o1(1,k2) );
    end



    % origin queue O1

    l_o1(1,k2+1) = partial_l_o1(1,k2+1) + partial_l_o1(2,k2+1);


    % %%% node model %%%
    %
    %   % flows entering from the node N1
    %
    Q_n1(1,k2) = q_o1(1,k2) * gamma_o1(1,k2); % Node N1 destination D1 class 1
    Q_n1(2,k2) = q_o1(1,k2) * gamma_o1(2,k2); % Node N1 destination D2 class 1

    q_m1(1,k2) = beta_m1(1,k2) * Q_n1(1,k2) + beta_m1(2,k2) * Q_n1(2,k2);

    % %%% freeway link model %%%
    %
    %   % gamma M1
    %
    for j=1:J_m(1,1) %alla gamma della sezione zero gli assegno il valore di quella della sezione 1
        if ( partial_rho_m1(j,1,k2) == 0 && rho_m1(1,k2) == 0 )
            gamma_m1(j,1,k2) = 0;
        else
            gamma_m1(j,1,k2) = partial_rho_m1(j,1,k2) / rho_m1(1,k2);
        end
    end
    %
    for j=1:J_m(1,1)
        for i=2:N_m(1,1)+1
            if ( partial_rho_m1(j,i-1,k2) == 0 && rho_m1(i-1,k2) == 0 )
                gamma_m1(j,i,k2) = 0;
            else
                gamma_m1(j,i,k2) = partial_rho_m1(j,i-1,k2) / rho_m1(i-1,k2);
            end
        end
    end


    % partial density link M1

    for j=1:J_m(1,1)  % for each destination
        for i=1:N_m(1,1) %for each section
            partial_rho_m1(j,i,k2+1) = partial_rho_m1(j,i,k2) + T / Delta_M(1,1) * ( gamma_m1(j,i,k2) * q_m1(i,k2) ...
                - gamma_m1(j,i+1,k2) * q_m1(i+1,k2) );
        end
    end

    % density link M1

    rho_m1(N_m(1,1)+1,k2) = rho_m2(1,k2);  %virtual density i+1



    for i=1:N_m(1,1)
        rho_m1(i,k2+1) = partial_rho_m1(1,i,k2+1)+ partial_rho_m1(2,i,k2+1);
    end


    %   %  mean speed link M1

    v_m1(1,k2) = v_o1(1,k2);

    for i=2:N_m(1,1)+1 % i=1 consider the virtual speed in i-1


        v_m1(i,k2+1) = v_m1(i,k2) + ( T / tau_1 ) * ( vf_1 * ( 1 - ( ( rho_m1(i-1,k2)) / rho_max )^ l1 )^m1 - v_m1(i,k2) )...
            +( T/Delta_M(1,1) ) * v_m1(i,k2) * ( v_m1(i-1,k2) - v_m1(i,k2) ) - ( ( nu_1 * T * ( rho_m1(i,k2) ...
            - rho_m1(i-1,k2) ) ) / ( Delta_M(1,1) * tau_1 * ( rho_m1(i-1,k2) + chi_1 ) ) );


        if ( v_m1(i,k2+1) < 0 )
            v_m1(i,k2+1) = 0;
        end

    end

    % flow link M1

    for i=2:N_m(1,1)+1
        q_m1(i,k2+1) = rho_m1(i-1,k2+1) * v_m1(i,k2+1);
    end

    % % accelerazioni segmental
    %
    %   for i=2:N_m(1,1)+1
    %     a_seg_m1(i-1,k) = ( ( v_m1(i,k+1) - v_m1(i,k) ) / T ) * conv_fact;
    %   end
    %
    % accelerazioni cross segmental

    %
    %   for i=2:N_m(1,1)+1
    %     if (i==N_m(1,1)+1)
    %         a_cross_m1(1,k) = ( ( v_m1(1,k+1) - v_m1(i,k) ) / T ) * conv_fact;
    %     else
    %         a_cross_m1(i-1,k) = ( ( v_m1(i+1,k+1) - v_m1(i,k) ) / T ) * conv_fact;
    %
    %     end
    %
    %   end

    %   % numero di veicoli
    %
    %     % n segmental
    %   for i=2:N_m(1,1)+1
    %     n_seg_m1(i-1,k) = Delta_M(1,1) * rho_m1(i,k) - T * q_m1(i,k);
    %   end
    %
    %     % n cross segmental
    %   for i=2:N_m(1,1)+1
    %     n_cross_m1(i-1,k) = T * q_m1(i,k);
    %   end

    %% on-ramp link model %%%
%%%$$$$
    % partial demand
    for j=1:J_r(3,1) % Destination reachable from o2
        partial_demand_o2(j,k2) = demand_o2(1,k2) * theta_o2(j,k2); % cars
    end

    l_o2(1,k2) = partial_l_o2(1,k2) + partial_l_o2(2,k2);

    q_o2(1,k2) = min( [ demand_o2(1,k2) + l_o2(1,k2) / T ; r_1_cap ;...
        r_1_cap * ( ( rho_max - rho_m2(1,k2) ) / (rho_max - rho_cr ) ) ]);


    for j=1:J_r(3,1)
        if( partial_l_o2(j,k2) == 0 && l_o2(1,k2)== 0)
            gamma_o2(j,k2) = theta_o2(j,k2);
        else
            gamma_o2(j,k2) = partial_l_o2(j,k2) / l_o2(1,k2);
        end

    end


    % partial on-ramp queue o2
    for j=1:J_r(3,1)
        partial_l_o2(j,k2+1) = partial_l_o2(j,k2) + T * ( demand_o2(1,k2) * theta_o2(j,k2) - gamma_o2(j,k2) * q_o2(1,k2) );

        if(partial_l_o2(j,k2+1)<0)
            partial_l_o2(j,k2+1)=0;
        end
    end


    %
    %   % acc sulla rampa
    %
    %     % acc arrival vehicles
    %       a_arr_o2(1,k) = ( ( v_idl_1(1,k+1) - v_on_1(1,k) ) / T ) * conv_fact;
    %
    %     % acc waiting vehicles
    %       a_w_o2(1,k) = ( ( v_idl_1(1,k+1) - v_idl_1(1,k) ) / T ) * conv_fact;
    %
    %     % acc leaving with stops vehicles
    %       a_ls_o2(1,k) = ( ( v_m1(1,k+1) - v_idl_1(1,k) ) / T ) * conv_fact;
    %
    %     % acc leaving without stops vehicles
    %       a_lns_o2(1,k) = ( ( v_m1(1,k+1) - v_on_1(1,k) ) / T ) * conv_fact;
    %
    % % numero veh auto
    % if ( q_o2(1,k) >= 0 ) && ( q_o2(1,k) <= (l_o2(1,k) / T) ), % scenario 1 auto
    %
    %             %n arrival vehicles
    %             n_arr_o2(1,k) = T * demand_o2(1,k);
    %
    %             % n waiting vehicles
    %             n_w_o2(1,k) = l_o2(1,k) - T * q_o2(1,k);
    %
    %             % n leaving with stops vehicles
    %             n_ls_o2(1,k) = T * q_o2(1,k);
    %
    %             % n leaving without stops vehicles
    %             n_lns_o2(1,k) = 0;
    %
    % elseif ( q_o2(1,k) <= (l_o2(1,k) / T + demand_o2(1,k) )) && ( q_o2(1,k) > ( l_o2(1,k) / T ) ) %scenario 2 auto
    %
    %             % n arrival vehicles
    %             n_arr_o2(1,k) = T * demand_o2(1,k) + l_o2(1,k) - T * q_o2(1,k);
    %
    %             % n waiting vehicles
    %             n_w_o2(1,k) = 0;
    %
    %             % n leaving with stops vehicles
    %             n_ls_o2(1,k) = l_o2(1,k);
    %
    %             % n leaving without stops vekicles
    %             n_lns_o2(1,k) = T * q_o2(1,k) - l_o2(1,k);
    % end
    %
    %%% node model %%%

    % flows entering from the node N2

    Q_n2(1,k2) = q_m1(N_m(1,1)+1,k2) * gamma_m1(1,k2) + q_o2(1,k2) * gamma_o2(1,k2); % Node N2 destination D1 class 1
    Q_n2(2,k2) = q_m1(N_m(1,1)+1,k2) * gamma_m1(2,k2) + q_o2(1,k2) * gamma_o2(2,k2); % Node N2 destination D2 class 1

    % outgoing traffic flow N2

    q_m2(1,k2) = beta_m2(1,k2) * Q_n2(1,k2) + beta_m2(2,k2) * Q_n2(2,k2);

    % %%% freeway link model %%%

    % gamma M2

    for j=1:J_m(2,1) %alla gamma della sezione zero gli assegno il valore di quella della sezione 1
        if ( partial_rho_m2(j,1,k2) == 0 && rho_m2(1,k2) == 0 )
            gamma_m2(j,1,k2) = 0;
        else
            gamma_m2(j,1,k2) = partial_rho_m2(j,1,k2) / rho_m2(1,k2);
        end
    end

    for j=1:J_m(2,1)
        for i=2:N_m(2,1)+1
            if ( partial_rho_m2(j,i-1,k2) == 0 && rho_m2(i-1,k2) == 0 )
                gamma_m2(j,i,k2) = 0;
            else
                gamma_m2(j,i,k2) = partial_rho_m2(j,i-1,k2) / rho_m2(i-1,k2);
            end
        end
    end


    % partial density link M2

    for j=1:J_m(2,1)  % for each destination
        for i=1:N_m(2,1) %for each section
            partial_rho_m2(j,i,k2+1) = partial_rho_m2(j,i,k2) + T / Delta_M(2,1) * ( gamma_m2(j,i,k2) * q_m2(i,k2) ...
                - gamma_m2(j,i+1,k2) * q_m2(i+1,k2) );
        end
    end

    % density link M2

    rho_m2(N_m(2,1)+1,k2) = ( rho_m3(1,k2)^2 + rho_m6(1,k2)^2 ) / ( rho_m3(1,k2) + rho_m6(1,k2) );  %virtual density i+1


    for i=1:N_m(2,1)
        rho_m2(i,k2+1) = partial_rho_m2(1,i,k2+1)+ partial_rho_m2(2,i,k2+1);
    end


    %   %  mean speed link M2

    v_m2(1,k2) = v_m1(N_m(1,1)+1,k2);

    for i=2:N_m(2,1)+1 % i=1 consider the virtual speed in i-1

        if i == 2 % considers the variation of the speed due to merging flow, rho i-1 , q_1_m1(i-1,k) from the node

            v_m2(i,k2+1) = v_m2(i,k2) + ( T / tau_1 ) * ( vf_1 * ( 1 - ( ( rho_m2(i-1,k2) ) / rho_max )^ l1 )^m1 - v_m2(i,k2) )...
                +( T/Delta_M(2,1) ) * v_m2(i,k2) * ( v_m2(i-1,k2) - v_m2(i,k2) ) - ( ( nu_1 * T * ( rho_m2(i,k2)...
                - rho_m2(i-1,k2) ) ) / ( Delta_M(2,1) * tau_1 * ( rho_m2(i-1,k2) + chi_1 ) ) )...
                -( ( delta_on_1 * T * v_m2(i,k2) * ( q_m2(i-1,k2) ) ) / ( Delta_M(2,1) * ( rho_m2(i-1,k2)...
                + chi_1 ) ) );

        else
            v_m2(i,k2+1) = v_m2(i,k2) + ( T / tau_1 ) * ( vf_1 * ( 1 - ( ( rho_m2(i-1,k2) ) / rho_max )^ l1 )^m1 - v_m2(i,k2) )...
                +( T/Delta_M(2,1) ) * v_m2(i,k2) * ( v_m2(i-1,k2) - v_m2(i,k2) ) - ( ( nu_1 * T * ( rho_m2(i,k2)...
                - rho_m2(i-1,k2) ) ) / ( Delta_M(2,1) * tau_1 * ( rho_m2(i-1,k2) + chi_1 ) ) );

        end
        if ( v_m2(i,k2+1) < 0 )
            v_m2(i,k2+1) = 0;
        end

    end

    % flow link M2

    for i=2:N_m(2,1)+1
        q_m2(i,k2+1) = rho_m2(i-1,k2+1) * v_m2(i,k2+1);
    end

    % accelerazioni segmental

    %   for i=2:N_m(2,1)+1
    %     a_seg_m2(i-1,k) = ( ( v_m2(i,k+1) - v_m2(i,k) ) / T ) * conv_fact;
    %   end
    % %
    % accelerazioni cross segmental

    %
    %   for i=2:N_m(2,1)+1
    %     if (i==N_m(2,1)+1)
    %         a_cross_m2_m3(1,k) = ( ( v_m3(1,k+1) - v_m2(i,k) ) / T ) * conv_fact;
    %         a_cross_m2_m6(1,k) = ( ( v_m6(1,k+1) - v_m2(i,k) ) / T ) * conv_fact;
    %     else
    %         a_cross_m2(i-1,k) = ( ( v_m2(i+1,k+1) - v_m2(i,k) ) / T ) * conv_fact;
    %     end
    %
    %   end

    %   % numero di veicoli
    %
    % n segmental
    %     for i=2:N_m(2,1)+1
    %         n_seg_m2(i-1,k) = Delta_M(2,1) * rho_m2(i,k) - T * q_m2(i,k);
    %     end

    % n cross segmental
    %     for i=2:N_m(2,1)+1
    %         if (i==N_m(2,1)+1)
    %             n_cross_m2_m3(i,k) = T * q_m3(1,k);
    %             n_cross_m2_m6(i,k) = T * q_m6(1,k);
    %         else
    %             n_cross_m2(i-1,k) = T * q_m2(i,k);
    %         end
    %     end

    % %%% node model %%%
    %
    %  flows entering from the node N3

    Q_n3(1,k2) = q_m2(N_m(2,1)+1,k2) * gamma_m2(1,N_m(2,1)+1,k2); % Node N3 destination D1 class 1
    Q_n3(2,k2) = q_m2(N_m(2,1)+1,k2) * gamma_m2(2,N_m(2,1)+1,k2); % Node N3 destination D2 class 1

    % outgoing traffic flow N2

    q_m3(1,k2) = beta_m3(1,k2) * Q_n3(1,k2) + beta_m3(2,k2) * Q_n3(2,k2);
    q_m6(1,k2) = beta_m6(1,k2) * Q_n3(1,k2) + beta_m6(2,k2) * Q_n3(2,k2);

    % %%% freeway link model %%%

    % gamma M3
    for j=1:J_m(3,1)
        if ( partial_rho_m3(j,1,k2) == 0 && rho_m3(1,k2) == 0 )
            gamma_m3(j,1,k2) = 0;
        else
            gamma_m3(j,1,k2) = partial_rho_m3(j,1,k2) / rho_m3(1,k2);
        end
    end

    for j=1:J_m(3,1)
        for i=2:N_m(3,1)+1
            if ( partial_rho_m3(j,i-1,k2) == 0 && rho_m3(i-1,k2) == 0 )
                gamma_m3(j,i,k2) = 0;
            else
                gamma_m3(j,i,k2) = partial_rho_m3(j,i-1,k2) / rho_m3(i-1,k2);
            end
        end
    end

    % partial density link M3

    for j=1:J_m(3,1)  % for each destination
        for i=1:N_m(3,1) %for each section
            partial_rho_m3(j,i,k2+1) = partial_rho_m3(j,i,k2) + T / Delta_M(3,1) * ( gamma_m3(j,i,k2) * q_m3(i,k2) ...
                - gamma_m3(j,i+1,k2) * q_m3(i+1,k2) );
        end
    end


    % density link M3

    rho_m3(N_m(3,1)+1,k2) = rho_m4(1,k2); %virtual density i+1

    for i=1:N_m(3,1)
        rho_m3(i,k2+1) = partial_rho_m3(1,i,k2+1);
    end

    % mean speed link M2

    v_m3(1,k2) = v_m2(N_m(2,1)+1,k2);

    for i=2:N_m(3,1)+1

        v_m3(i,k2+1) = v_m3(i,k2) + ( T / tau_1 ) * ( vf_1 * ( 1 - ( ( rho_m3(i-1,k2) ) / rho_max )^ l1 )^m1 - v_m3(i,k2) )...
            +( T/Delta_M(3,1) ) * v_m3(i,k2) * ( v_m3(i-1,k2) - v_m3(i,k2) ) - ( ( nu_1 * T * ( rho_m3(i,k2)...
            - rho_m3(i-1,k2) ) ) / ( Delta_M(3,1) * tau_1 * ( rho_m3(i-1,k2) + chi_1 ) ) );

        if ( v_m3(i,k2+1) < 0 )
            v_m3(i,k2+1) = 0;
        end

    end

    % flow link M3
    for i=2:N_m(3,1)+1
        q_m3(i,k2+1) = rho_m3(i-1,k2+1) * v_m3(i,k2+1);
    end

    %         % accelerazioni segmental
    %
    %       for i=2:N_m(3,1)+1
    %         a_seg_m3(i-1,k) = ( ( v_m3(i,k+1) - v_m3(i,k) ) / T ) * conv_fact;
    %       end
    %
    %      % accelerazioni cross segmental
    %
    %       for i=2:N_m(2,1)+1
    %           if (i== N_m(3,1)+1 )
    %               a_cross_m3(i-1,k) = ( ( v_m4(1,k+1) - v_m3(i,k) ) / T ) * conv_fact;
    %           else
    %               a_cross_m3(i-1,k) = ( ( v_m3(i+1,k+1) - v_m3(i,k) ) / T ) * conv_fact;
    %           end
    %       end
    %
    % numero di veicoli

    %         % n segmental
    %       for i=2:N_m(3,1)+1
    %         n_seg_m3(i-1,k) = Delta_M(3,1) * rho_m3(i,k) - T * q_m3(i,k);
    %       end
    %
    %         % n cross segmental
    %       for i=2:N_m(2,1)+1
    %         n_cross_m3(i-1,k) = T * q_m3(i,k);
    %       end

    % %  %%% freeway link model %%%
    %
    
    % gamma M6
    for j=1:J_m(6,1)
        if ( partial_rho_m6(j,1,k2) == 0 && rho_m6(1,k2) == 0 )
            gamma_m6(j,1,k2) = 0;
        else
            gamma_m6(j,1,k2) = partial_rho_m6(j,1,k2) / rho_m6(1,k2);
        end
    end
    
      for j=1:J_m(6,1)
          for i=2:N_m(6,1)+1
              if ( partial_rho_m6(j,i-1,k2) == 0 && rho_m6(i-1,k2) == 0 )
                  gamma_m6(j,i,k2) = 0;
              else
                  gamma_m6(j,i,k2) = partial_rho_m6(j,i-1,k2) / rho_m6(i-1,k2);
              end
          end
      end
    
    
      % partial density link M6
    
      for j=1:J_m(6,1)  % for each destination
          for i=1:N_m(6,1) %for each section
              partial_rho_m6(j,i,k2+1) = partial_rho_m6(j,i,k2) + T / Delta_M(6,1) * ( gamma_m6(j,i,k2) * q_m6(i,k2) ...
                  - gamma_m6(j,i+1,k2) * q_m6(i+1,k2) );
          end
      end
    
    
    
      % density link M6
    
      rho_m6(N_m(6,1)+1,k2) = rho_m7(1,k2); %virtual density i+1
      for i=1:N_m(6,1)

          rho_m6(i,k2+1) = partial_rho_m6(1,i,k2+1) + partial_rho_m6(2,i,k2+1);
      end
    
    
      % mean speed link M6
    
          v_m6(1,k2) = v_m2(N_m(2,1)+1,k2);
    
      for i=2:N_m(6,1)+1
    
          v_m6(i,k2+1) = v_m6(i,k2) + ( T / tau_1 ) * ( vf_1 * ( 1 - ( ( rho_m6(i-1,k2) ) / rho_max )^ l1 )^m1 - v_m6(i,k2) )...
                           +( T/Delta_M(6,1) ) * v_m6(i,k2) * ( v_m6(i-1,k2) - v_m6(i,k2) ) - ( ( nu_1 * T * ( rho_m6(i,k2) ...
                           - rho_m6(i-1,k2) ) ) / ( Delta_M(6,1) * tau_1 * ( rho_m6(i-1,k2) + chi_1 ) ) );
    
        if ( v_m6(i,k2+1) < 0 )
            v_m6(i,k2+1) = 0;
        end
    
    
      end
    
      % flow link M6
      for i=2:N_m(6,1)+1
          q_m6(i,k2+1) = rho_m6(i-1,k2+1) * v_m6(i,k2+1);
      end

      % accelerazioni segmental

%       for i=2:N_m(6,1)+1
%           a_seg_m6(i-1,k) = ( ( v_m6(i,k+1) - v_m6(i,k) ) / T ) * conv_fact;
%       end

     % accelerazioni cross segmental
    
%      for i=2:N_m(6,1)+1
%          if (i==N_m(6,1)+1)
%              a_cross_m6(i-1,k) = ( ( v_m7(1,k+1) - v_m6(i,k) ) / T ) * conv_fact;
%          else
%              a_cross_m6(i-1,k) = ( ( v_m6(i+1,k+1) - v_m6(i,k) ) / T ) * conv_fact;
%          end
%      end

      % numero di veicoli
    
        % n segmental
%         for i=2:N_m(6,1)+1
%             n_seg_m6(i-1,k) = Delta_M(6,1) * rho_m6(i,k) - T * q_m6(i,k);
%         end

        % n cross segmental
%         for i=2:N_m(6,1)+1
%             n_cross_m6(i-1,k) = T * q_m6(i,k);
%         end
    
        %% on-ramp link model %%%
    
       % partial demand

       for j=1:J_r(2,1) % Destination reachable from o5
           partial_demand_o5(j,k2) = demand_o5(1,k2) * theta_o5(j,k2); % cars
       end

       l_o5(1,k2) =  partial_l_o5(1,k2) + partial_l_o5(2,k2);

       q_o5(1,k2) = min( [ demand_o5(1,k2) + l_o5(1,k2) / T ; Q_max ;...
           Q_max* ( ( rho_max - rho_m7(1,k2) ) / (rho_max - rho_cr ) ) ]);

    
       for j=1:J_r(2,1)
           if( partial_l_o5(j,k2) == 0 && l_o5(1,k2)== 0)
               gamma_o5(j,k2) = theta_o5(j,k2);
           else
               gamma_o5(j,k2) = partial_l_o5(j,k2) / l_o5(1,k2);
           end

       end


       % partial on-ramp queue o5
       for j=1:J_r(2,1)
           partial_l_o5(j,k2+1) = partial_l_o5(j,k2) + T * ( demand_o5(1,k2) * theta_o5(j,k2) - gamma_o5(j,k2) * q_o5(1,k2) );

           if(partial_l_o5(j,k2+1)<0)
               partial_l_o5(j,k2+1)=0;
           end
       end

    
    
      % acc sulla rampa
%     
%         % acc arrival vehicles
%         a_arr_o5(1,k) = ( ( v_idl_1(1,k+1) - v_on_1(1,k) ) / T ) * conv_fact;
%         % acc waiting vehicles
%         a_w_o5(1,k) = ( ( v_idl_1(1,k+1) - v_idl_1(1,k) ) / T ) * conv_fact;
%         % acc leaving with stops vehicles
%         a_ls_o5(1,k) = ( ( v_m7(1,k+1) - v_idl_1(1,k) ) / T ) * conv_fact;
%         % acc leaving without stops vehicles
%         a_lns_o5(1,k) = ( ( v_m7(1,k+1) - v_on_1(1,k) ) / T ) * conv_fact;

    % numero veh auto

%     if ( q_o5(1,k) >= 0 ) && ( q_o5(1,k) <= (l_o5(1,k) / T) ) % scenario 1 auto
% 
% %         n arrival vehicles
%         n_arr_o5(1,k) = T * demand_o5(1,k);
% 
% %         n waiting vehicles
%         n_arr_o5(1,k) = l_o5(1,k) - T * q_o5(1,k);
% 
% %         n leaving with stops vehicles
%         n_ls_o5(1,k) = T * q_o5(1,k);
% 
% %         n leaving without stops vehicles
%         n_lns_o5(1,k) = 0;
% 
%     elseif ( q_o5(1,k) <= (l_o5(1,k) / T + demand_o5(1,k) )) && ( q_o5(1,k) > ( l_o5(1,k) / T ) ) %scenario 2 auto
% 
%         % n arrival vehicles
%         n_arr_o5(1,k) = T * demand_o5(1,k) + l_o5(1,k) - T * q_o5(1,k);
% 
%         % n waiting vehicles
%         n_arr_o5(1,k) = 0;
% 
%         % n leaving with stops vehicles
%         n_ls_o5(1,k) = l_o5(1,k);
% 
%         % n leaving without stops vekicles
%         n_lns_o5(1,k) = T * q_o5(1,k) - l_o5(1,k);
%     end
    
    
      % node model %%%
    
      % flows entering from the nodes N7
    
          Q_n7(1,k2) = q_m6(N_m(6,1)+1,k2) * gamma_m6(1,N_m(6,1)+1,k2) + q_o5(1,k2) * gamma_o5(1,k2); % Node N7 destination D1 class 1
          Q_n7(2,k2) = q_m6(N_m(6,1)+1,k2) * gamma_m6(2,N_m(6,1)+1,k2) + q_o5(1,k2) * gamma_o5(2,k2); % Node N7 destination D2 class 1
    
      % outgoing traffic flow N7
    
          q_m7(1,k2) = beta_m7(1,k2) * Q_n7(1,k2) + beta_m7(2,k2) * Q_n7(2,k2); %flusso diretto a D1 che utilizzano il link m7
    
          %% freeway link model %%%

          % gamma M7

          for j=1:J_m(7,1) %alla gamma della sezione zero gli assegno il valore di quella della sezione 1
              if ( partial_rho_m7(j,1,k2) == 0 && rho_m7(1,k2) == 0 )
                  gamma_m7(j,1,k2) = 0;
              else
                  gamma_m7(j,1,k2) = partial_rho_m7(j,1,k2) / rho_m7(1,k2);
              end
          end

          for j=1:J_m(7,1)
              for i=2:N_m(7,1)+1
                  if ( partial_rho_m7(j,i-1,k2) == 0 && rho_m7(i-1,k2) == 0 )
                      gamma_m7(j,i,k2) = 0;
                  else
                      gamma_m7(j,i,k2) = partial_rho_m7(j,i-1,k2) / rho_m7(i-1,k2);
                  end
              end
          end


          % partial density link M7

          for j=1:J_m(7,1)  % for each destination
              for i=1:N_m(7,1) %for each section
                  partial_rho_m7(j,i,k2+1) = partial_rho_m7(j,i,k2) + T / Delta_M(7,1) * ( gamma_m7(j,i,k2) * q_m7(i,k2) ...
                      - gamma_m7(j,i+1,k2) * q_m7(i+1,k2) );
              end
          end

          % density link M7

          rho_m7(N_m(7,1)+1,k2) =  ( rho_m8(1,k2)^2 + rho_m9(1,k2)^2 ) / ( rho_m8(1,k2) + rho_m9(1,k2) ); %virtual density i+1

          for i=1:N_m(7,1)
              rho_m7(i,k2+1) = partial_rho_m7(1,i,k2+1) + partial_rho_m7(2,i,k2+1);
          end

          if ( rho_m7(1,k2) == 0 && rho_m7(1,k2) == 0)
              rho_m7(N_m(7,1)+1,k2) = 0;
          end



      %  mean speed link M7
    
         v_m7(1,k2) = v_m6(N_m(6,1)+1,k2);
    %      v_2_m7_nc(1,k) = v_2_m6_nc(N_m(6,1)+1,k);
    
    for i=2:N_m(7,1)+1 % i=1 consider the virtual speed in i-1

        if i == 2 % considers the variation of the speed due to merging flow, rho i-1 , q_1_m1(i-1,k) from the node

            v_m7(i,k2+1) = v_m7(i,k2) + ( T / tau_1 ) * ( vf_1 * ( 1 - ( ( rho_m7(i-1,k2)) / rho_max )^ l1 )^m1 - v_m7(i,k2) )...
                +( T/Delta_M(7,1) ) * v_m7(i,k2) * ( v_m7(i-1,k2) - v_m7(i,k2) ) - ( ( nu_1 * T * ( rho_m7(i,k2) ...
                - rho_m7(i-1,k2) ) ) / ( Delta_M(7,1) * tau_1 * ( rho_m7(i-1,k2) + chi_1 ) ) )...
                -( ( delta_on_1 * T * v_m7(i,k2) * ( q_m7(i-1,k2) ) ) / ( Delta_M(7,1) * ( rho_m7(i-1,k2)...
                + chi_1 ) ) );

        else

            v_m7(i,k2+1) = v_m7(i,k2) + ( T / tau_1 ) * ( vf_1 * ( 1 - ( ( rho_m7(i-1,k2) ) / rho_max )^ l1 )^m1 - v_m7(i,k2) )...
                +( T/Delta_M(7,1) ) * v_m7(i,k2) * ( v_m7(i-1,k2) - v_m7(i,k2) ) - ( ( nu_1 * T * ( rho_m7(i,k2) ...
                - rho_m7(i-1,k2)  ) ) / ( Delta_M(7,1) * tau_1 * ( rho_m7(i-1,k2) + chi_1 ) ) );

        end

        if ( v_m7(i,k2+1) < 0 )
            v_m7(i,k2+1) = 0;
        end


    end

    % flow link M7

    for i=2:N_m(7,1)+1
        q_m7(i,k2+1) = rho_m7(i-1,k2+1) * v_m7(i,k2+1);
    end

    % accelerazioni segmental

%     for i=2:N_m(7,1)+1
%         a_seg_m7(i-1,k) = ( ( v_m7(i,k+1) - v_m7(i,k) ) / T ) * conv_fact;
%     end

    % accelerazioni cross segmental

% 
%     for i=2:N_m(7,1)+1
%         if (i==N_m(7,1)+1)
%             a_cross_m7_m8(1,k) = ( ( v_m8(1,k+1) - v_m7(i,k) ) / T ) * conv_fact;
%             a_cross_m7_m9(1,k) = ( ( v_m9(1,k+1) - v_m7(i,k) ) / T ) * conv_fact;
%         else
%             a_cross_m7(i-1,k) = ( ( v_m7(i+1,k+1) - v_m7(i,k) ) / T ) * conv_fact;
%         end
%     end

    % numero di veicoli

    % n segmental
%     for i=2:N_m(7,1)+1
%         n_seg_m7(i-1,k) = Delta_M(7,1) * rho_m7(i,k) - T * q_m7(i,k);
%     end

    % n cross segmental
%     for i=2:N_m(7,1)+1
%         if (i==N_m(7,1)+1)
%             n_cross_m7_m8(i,k) = T * q_m8(1,k);
%             n_cross_m7_m9(i,k) = T * q_m9(1,k);
%         else
%             n_cross_m7(i-1,k) = T * q_m7(i,k);
%         end
%     end

       % node model %%%
    
      % flows entering from the nodes N8
    
          Q_n8(1,k2) = q_m7(N_m(7,1)+1,k2) * gamma_m7(1,N_m(7,1)+1,k2); % Node N8 destination D1 class 1
          Q_n8(2,k2) = q_m7(N_m(7,1)+1,k2) * gamma_m7(2,N_m(7,1)+1,k2); % Node N8 destination D1 class 1
    
      % outgoing traffic flow N8
    
          q_m8(1,k2) = beta_m8(1,k2) * Q_n8(1,k2) + beta_m8(2,k2) * Q_n8(2,k2); %flusso diretto a D1 che utilizzano il link m8
          q_m9(1,k2) = beta_m9(1,k2) * Q_n8(1,k2) + beta_m9(2,k2) * Q_n8(2,k2); %flusso diretto a D2 che utilizzano il link m9
    
    %%% freeway link model %%%
    
      % gamma M8
    
      for j=1:J_m(8,1) %alla gamma della sezione zero gli assegno il valore di quella della sezione 1
          if ( partial_rho_m8(j,1,k2) == 0 && rho_m8(1,k2) == 0 )
              gamma_m8(j,1,k2) = 0;
          else
              gamma_m8(j,1,k2) = partial_rho_m8(j,1,k2) / rho_m8(1,k2);
          end
      end

      for j=1:J_m(8,1)
          for i=2:N_m(8,1)+1
              if ( partial_rho_m8(j,i-1,k2) == 0 && rho_m8(i-1,k2) == 0 )
                  gamma_m8(j,i,k2) = 0;
              else
                  gamma_m8(j,i,k2) = partial_rho_m8(j,i-1,k2) / rho_m8(i-1,k2);
              end
          end
      end
    
    
       % partial density link M8
    
      for j=1:J_m(8,1)  % for each destination
       for i=1:N_m(8,1) %for each section
          partial_rho_m8(j,i,k2+1) = partial_rho_m8(j,i,k2) + T / Delta_M(8,1) * ( gamma_m8(j,i,k2) * q_m8(i,k2) ...
                                         - gamma_m8(j,i+1,k2) * q_m8(i+1,k2) );
       end
      end
    
      % density link M8

      rho_m8(N_m(8,1)+1,k2) =  rho_m5(1,k2); %virtual density i+1

      for i=1:N_m(8,1)
          rho_m8(i,k2+1) = partial_rho_m8(1,i,k2+1);
      end

      if ( rho_m8(1,k2) == 0 && rho_m8(1,k2) == 0)
          rho_m8(N_m(8,1)+1,k2) = 0;
      end



      %  mean speed link M8

      v_m8(1,k2) = v_m7(N_m(7,1)+1,k2);

      for i=2:N_m(8,1)+1 % i=1 consider the virtual speed in i-1


          v_m8(i,k2+1) = v_m8(i,k2) + ( T / tau_1 ) * ( vf_1 * ( 1 - ( ( rho_m8(i-1,k2) ) / rho_max )^ l1 )^m1 - v_m8(i,k2) )...
              +( T/Delta_M(8,1) ) * v_m8(i,k2) * ( v_m8(i-1,k2) - v_m8(i,k2) ) - ( ( nu_1 * T * ( rho_m8(i,k2) ...
              - rho_m8(i-1,k2) ) ) / ( Delta_M(8,1) * tau_1 * ( rho_m8(i-1,k2) + chi_1 ) ) );

          if ( v_m8(i,k2+1) < 0 )
              v_m8(i,k2+1) = 0;
          end

      end

      % flow link M8

      for i=2:N_m(8,1)+1
          q_m8(i,k2+1) = rho_m8(i-1,k2+1) * v_m8(i,k2+1);
      end
    
    
    %   % accelerazioni segmental

%     for i=2:N_m(8,1)+1
%         a_seg_m8(i-1,k) = ( ( v_m8(i,k+1) - v_m8(i,k) ) / T ) * conv_fact;
%     end
% 

    %  % accelerazioni cross segmental

%     for i=2:N_m(8,1)+1
%         if (i==N_m(8,1)+1)
%             a_cross_m8(i-1,k) = ( ( v_m5(1,k+1) - v_m8(i,k) ) / T ) * conv_fact;
%         else
%             a_cross_m8(i-1,k) = ( ( v_m8(i+1,k+1) - v_m8(i,k) ) / T ) * conv_fact;
%         end
%     end

    %   % numero di veicoli
% 
%     % n segmental
%     for i=2:N_m(8,1)+1
%         n_seg_m8(i-1,k) = Delta_M(8,1) * rho_m8(i,k) - T * q_m8(i,k);
%     end
% 
%     % n cross segmental
%     for i=2:N_m(8,1)+1
%         n_cross_m8(i-1,k) = T * q_m8(i,k);
%     end


    %   %%% freeway link model %%%
    
      % gamma M9
    
      for j=1:J_m(9,1) %alla gamma della sezione zero gli assegno il valore di quella della sezione 1
          if ( partial_rho_m9(j,1,k2) == 0 && rho_m9(1,k2) == 0 )
              gamma_m9(j,1,k2) = 0;
          else
              gamma_m9(j,1,k2) = partial_rho_m9(j,1,k2) / rho_m9(1,k2);
          end
      end

      for j=1:J_m(9,1)
          for i=2:N_m(9,1)+1
              if ( partial_rho_m9(j,i-1,k2) == 0 && rho_m9(i-1,k2) == 0 )
                  gamma_m9(j,i,k2) = 0;
              else
                  gamma_m9(j,i,k2) = partial_rho_m9(j,i-1,k2) / rho_m9(i-1,k2);
              end
          end
      end
    
       % partial density link M9

       for j=1:J_m(9,1)  % for each destination
           for i=1:N_m(9,1) %for each section
               partial_rho_m9(j,i,k2+1) = partial_rho_m9(j,i,k2) + T / Delta_M(9,1) * ( gamma_m9(j,i,k2) * q_m9(i,k2) ...
                   - gamma_m9(j,i+1,k2) * q_m9(i+1,k2) );
           end
       end
    
      % density link M9
    
         rho_m9(N_m(9,1)+1,k2) =  rho_m11(1,k2); %virtual density i+1
    
       for i=1:N_m(9,1)
           rho_m9(i,k2+1) = partial_rho_m9(1,i,k2+1);
      end
    
         if ( rho_m9(1,k2) == 0 && rho_m9(1,k2) == 0)
             rho_m9(N_m(9,1)+1,k2) = 0;
         end
    
      %  mean speed link M9
    
         v_m9(1,k2) =  v_m7(N_m(7,1),k2);
      for i=2:N_m(9,1)+1 % i=1 consider the virtual speed in i-1
    
         v_m9(i,k2+1) = v_m9(i,k2) + ( T / tau_1 ) * ( vf_1 * ( 1 - ( ( rho_m9(i-1,k2) ) / rho_max )^ l1 )^m1 - v_m9(i,k2) )...
                         +( T/Delta_M(9,1) ) * v_m9(i,k2) * ( v_m9(i-1,k2) - v_m9(i,k2) ) - ( ( nu_1 * T * ( rho_m9(i,k2) ...
                         - rho_m9(i-1,k2) ) ) / ( Delta_M(9,1) * tau_1 * ( rho_m9(i-1,k2) + chi_1 ) ) );
    
        if ( v_m9(i,k2+1) < 0 )
            v_m9(i,k2+1) = 0;
        end
    
    
      end
    
      % flow link M9
    
      for i=2:N_m(9,1)+1
         q_m9(i,k2+1) = rho_m9(i-1,k2+1) * v_m9(i,k2+1);
      end
    
    
%     %   % accelerazioni segmental
%     
%       for i=2:N_m(9,1)+1
%         a_seg_1_m9(i-1,k) = ( ( v_m9(i,k+1) - v_m9(i,k) ) / T ) * conv_fact;
%       end
%     
%     %  % accelerazioni cross segmental
%     
%       for i=2:N_m(9,1)+1
%           if (i==N_m(9,1)+1)
%               a_cross_m9(i-1,k) = ( ( v_m10(1,k+1) - v_m9(i,k) ) / T ) * conv_fact;
%           else
%               a_cross_m9(i-1,k) = ( ( v_m9(i+1,k+1) - v_m9(i,k) ) / T ) * conv_fact;
%           end
%       end
%     
%     %   % numero di veicoli
%     
%         % n segmental
%       for i=2:N_m(9,1)+1
%         n_seg_1_m9(i-1,k) = Delta_M(9,1) * rho_m9(i,k) - T * q_m9(i,k);
%       end
%     
%         % n cross segmental
%         for i=2:N_m(9,1)+1
%             n_cross_m9(i-1,k) = T * q_m9(i,k);
%         end

%% on-ramp link model %%%

% partial demand
for j=1:J_r(1,1) % Destination reachable from o3
    partial_demand_o3(j,k2) = demand_o3(1,k2) * theta_o3(j,k2); % cars
end

l_1_o3(1,k2) = partial_l_o3(1,k2) ;

q_o3(1,k2) = min( [ demand_o3(1,k2) + l_1_o3(1,k2) / T ; r_1_cap ;...
    r_1_cap * ( ( rho_max - rho_m4(1,k2) ) / (rho_max - rho_cr ) ) ]);


for j=1:J_r(1,1)
    if( partial_l_o3(j,k2) == 0 && l_1_o3(1,k2)== 0)
        gamma_o3(j,k2) = theta_o3(j,k2);
    else
        gamma_o3(j,k2) = partial_l_o3(j,k2) / l_1_o3(1,k2);
    end

end



% partial on-ramp queue o3
for j=1:J_r(1,1)
    partial_l_o3(j,k2+1) = partial_l_o3(j,k2) + T * ( demand_o3(1,k2) * theta_o3(j,k2) - gamma_o3(j,k2) * q_o3(1,k2) );

    if(partial_l_o3(j,k2+1) <0)
        partial_l_o3(j,k2+1) =0;
    end
end



% acc sulla rampa

% % acc arrival vehicles
% a_arr_o3(1,k) = ( ( v_idl_1(1,k+1) - v_on_1(1,k) ) / T ) * conv_fact;
% 
% % acc waiting vehicles
% a_w_o3(1,k) = ( ( v_idl_1(1,k+1) - v_idl_1(1,k) ) / T ) * conv_fact;
% 
% % acc leaving with stops vehicles
% a_ls_o3(1,k) = ( ( v_m4(1,k+1) - v_idl_1(1,k) ) / T ) * conv_fact;
% 
% % acc leaving without stops vehicles
% a_lns_o3(1,k) = ( ( v_m4(1,k+1) - v_on_1(1,k) ) / T ) * conv_fact;

% numero veh auto
% if ( q_o3(1,k) >= 0 ) && ( q_o3(1,k) <= (l_1_o3(1,k) / T) ) % scenario 1 auto
% 
%     %n arrival vehicles
%     n_arr_o3(1,k) = T * demand_o3(1,k);
% 
%     % n waiting vehicles
%     n_w_o3(1,k) = l_1_o3(1,k) - T * q_o3(1,k);
% 
%     % n leaving with stops vehicles
%     n_ls_o3(1,k) = T * q_o3(1,k);
% 
%     % n leaving without stops vehicles
%     n_lns_o3(1,k) = 0;
% 
% elseif ( q_o3(1,k) <= (l_1_o3(1,k) / T + demand_o3(1,k) )) && ( q_o3(1,k) > ( l_1_o3(1,k) / T ) ) %scenario 2 auto
% 
%     % n arrival vehicles
%     n_arr_o3(1,k) = T * demand_o3(1,k) + l_1_o3(1,k) - T * q_o3(1,k);
% 
%     % n waiting vehicles
%     n_w_o3(1,k) = 0;
% 
%     % n leaving with stops vehicles
%     n_ls_o3(1,k) = l_1_o3(1,k);
% 
%     % n leaving without stops vekicles
%     n_lns_o3(1,k) = T * q_o3(1,k) - l_1_o3(1,k);
% end



% node model %%%

% flows entering from the nodes N4
Q_n4(1,k2) = q_m3(N_m(3,1)+1,k2) * gamma_m3(1,N_m(3,1)+1,k2) + q_o3(1,k2) * gamma_o3(1,k2); % Node N3 destination D1 class 1

% outgoing traffic flow N4
q_m4(1,k2) = beta_m4(1,k2) * Q_n4(1,k2); %flusso diretto a D1 che utilizzano il link m4

%%% freeway link model %%%


% gamma M4

for j=1:J_m(4,1) %alla gamma della sezione zero gli assegno il valore di quella della sezione 1
    if ( partial_rho_m4(j,1,k2) == 0 && rho_m4(1,k2) == 0 )
        gamma_m4(j,1,k2) = 0;
    else
        gamma_m4(j,1,k2) = partial_rho_m4(j,1,k2) / rho_m4(1,k2);
    end
end

for j=1:J_m(4,1)
    for i=2:N_m(4,1)+1
        if ( partial_rho_m4(j,i-1,k2) == 0 && rho_m4(i-1,k2) == 0 )
            gamma_m4(j,i,k2) = 0;
        else
            gamma_m4(j,i,k2) = partial_rho_m4(j,i-1,k2) / rho_m4(i-1,k2);
        end
    end
end

% partial density link M4

for j=1:J_m(4,1)  % for each destination
    for i=1:N_m(4,1) %for each section
        partial_rho_m4(j,i,k2+1) = partial_rho_m4(j,i,k2) + T / Delta_M(4,1) * ( gamma_m4(j,i,k2) * q_m4(i,k2) ...
            - gamma_m4(j,i+1,k2) * q_m4(i+1,k2) );

    end
end

% density link M4

rho_m4(N_m(4,1)+1,k2) = rho_m5(1,k2); %virtual density i+1

for i=1:N_m(4,1)
    rho_m4(i,k2+1) = partial_rho_m4(1,i,k2+1) ;

end

if ( rho_m4(1,k2) == 0 && rho_m4(1,k2) == 0)
    rho_m4(N_m(4,1)+1,k2) = 0;
end




%  mean speed link M4

v_m4(1,k2) = v_m3(N_m(2,1)+1,k2);

for i=2:N_m(4,1)+1 % i=1 consider the virtual speed in i-1

    if i == 2 % considers the variation of the speed due to merging flow, rho i-1 , q_1_m1(i-1,k) from the node

        v_m4(i,k2+1) = v_m4(i,k2) + ( T / tau_1 ) * ( vf_1 * ( 1 - ( ( rho_m4(i-1,k2) ) / rho_max )^ l1 )^m1 - v_m4(i,k2) )...
            +( T/Delta_M(4,1) ) * v_m4(i,k2) * ( v_m4(i-1,k2) - v_m4(i,k2) ) - ( ( nu_1 * T * ( rho_m4(i,k2) ...
            - rho_m4(i-1,k2) ) ) / ( Delta_M(4,1) * tau_1 * ( rho_m4(i-1,k2) + chi_1 ) ) )...
            -( ( delta_on_1 * T * v_m4(i,k2) * ( q_m4(i-1,k2)  ) ) / ( Delta_M(3,1) * ( rho_m4(i-1,k2)...
            + chi_1 ) ) );


    elseif (i==4 || i==5 && k2>=180 && k2<(550))

        v_m4(i,k2+1) = v_m4(i,k2) + ( T / tau_1 ) * ( vf_1 * ( 1 - ( ( rho_m4(i-1,k2)  ) /300 )^ l1 )^m1 - v_m4(i,k2) )...
            +( T/Delta_M(4,1) ) * v_m4(i,k2) * ( v_m4(i-1,k2) - v_m4(i,k2) ) - ( ( nu_1 * T * ( rho_m4(i,k2) ...
            - rho_m4(i-1,k2) ) ) / ( Delta_M(4,1) * tau_1 * ( rho_m4(i-1,k2) + chi_1 ) ) );

    else

        v_m4(i,k2+1) = v_m4(i,k2) + ( T / tau_1 ) * ( vf_1 * ( 1 - ( ( rho_m4(i-1,k2) ) / rho_max )^ l1 )^m1 - v_m4(i,k2) )...
            +( T/Delta_M(4,1) ) * v_m4(i,k2) * ( v_m4(i-1,k2) - v_m4(i,k2) ) - ( ( nu_1 * T * ( rho_m4(i,k2) ...
            - rho_m4(i-1,k2) ) ) / ( Delta_M(4,1) * tau_1 * ( rho_m4(i-1,k2) + chi_1 ) ) );

    end


    if ( v_m4(i,k2+1) < 0 )
        v_m4(i,k2+1) = 0;
    end



end

% flow link M4

for i=2:N_m(4,1)+1
    q_m4(i,k2+1) = rho_m4(i-1,k2+1) * v_m4(i,k2+1);
end

% accelerazioni segmental
% 
% for i=2:N_m(4,1)+1
%     a_seg_m4(i-1,k) = ( ( v_m4(i,k+1) - v_m4(i,k) ) / T ) * conv_fact;
% end
% 
% % accelerazioni cross segmental
% 
% for i=2:N_m(4,1)+1
%     if (i==N_m(4,1)+1)
%         a_cross_m4(i-1,k) = ( ( v_m5(1,k+1) - v_m4(i,k) ) / T ) * conv_fact;
%     else
%         a_cross_m4(i-1,k) = ( ( v_m4(i+1,k+1) - v_m4(i,k) ) / T ) * conv_fact;
%     end
% end
% 
% % numero di veicoli
% 
% % n segmental
% for i=2:N_m(4,1)+1
%     n_seg_m4(i-1,k) = Delta_M(4,1) * rho_m4(i,k) - T * q_m4(i,k);
% end
% 
% % n cross segmental
% for i=2:N_m(4,1)+1
%     n_cross_m4(i-1,k) = T * q_m4(i,k);
% end

% node model %%%

% flows entering from the nodes N5
Q_n5(1,k2) = q_m4(N_m(4,1)+1,k2) * gamma_m4(1,N_m(4,1)+1,k2) + q_m8(N_m(8,1)+1,k2) * gamma_m8(1,N_m(8,1)+1,k2); % Node N4 destination D1 class 1

% outgoing traffic flow N5
q_m5(1,k2) = beta_m5(1,k2) * Q_n5(1,k2); %flusso diretto a D1 che utilizzano il link m5


%%% freeway link model %%%

% gamma M5

for j=1:J_m(5,1) %alla gamma della sezione zero gli assegno il valore di quella della sezione 1
    if ( partial_rho_m5(j,1,k2) == 0 && rho_m5(1,k2) == 0 )
        gamma_m5(j,1,k2) = 0;
    else
        gamma_m5(j,1,k2) = partial_rho_m5(j,1,k2) / rho_m5(1,k2);
    end
end

for j=1:J_m(5,1)
    for i=2:N_m(5,1)+1
        if ( partial_rho_m5(j,i-1,k2) == 0 && rho_m5(i-1,k2) == 0 )
            gamma_m5(j,i,k2) = 0;
        else
            gamma_m5(j,i,k2) = partial_rho_m5(j,i-1,k2) / rho_m5(i-1,k2);
        end
    end
end

% partial density link M5

for j=1:J_m(5,1)  % for each destination
    for i=1:N_m(5,1) %for each section
        partial_rho_m5(j,i,k2+1) = partial_rho_m5(j,i,k2) + T / Delta_M(5,1) * ( gamma_m5(j,i,k2) * q_m5(i,k2) ...
            - gamma_m5(j,i+1,k2) * q_m5(i+1,k2) );
    end
end

% density link M5

rho_m5(N_m(5,1)+1,k2) =  rho_m10(1,k2); %virtual density i+1

for i=1:N_m(5,1)
    rho_m5(i,k2+1) = partial_rho_m5(1,i,k2+1);
end

if ( rho_m5(1,k2) == 0 && rho_m5(1,k2) == 0)
    rho_m5(N_m(5,1)+1,k2) = 0;
end


%  mean speed link M5

v_m5(1,k2) = ( (  v_m4(N_m(4,1)+1,k2) * q_m4(N_m(4,1)+1,k2) + v_m8(N_m(8,1),k2) * q_m8(N_m(8,1)+1,k2) ) ) / (q_m4(N_m(4,1)+1,k2) + q_m8(N_m(8,1)+1,k2) );

for i=2:N_m(5,1)+1 % i=1 consider the virtual speed in i-1

    v_m5(i,k2+1) = v_m5(i,k2) + ( T / tau_1 ) * ( vf_1 * ( 1 - ( ( rho_m5(i-1,k2) ) / rho_max )^ l1 )^m1 - v_m5(i,k2) )...
        +( T/Delta_M(5,1) ) * v_m5(i,k2) * ( v_m5(i-1,k2) - v_m5(i,k2) ) - ( ( nu_1 * T * ( rho_m5(i,k2) ...
        - rho_m5(i-1,k2) ) ) / ( Delta_M(5,1) * tau_1 * ( rho_m5(i-1,k2)  + chi_1 ) ) );

    if ( v_m5(i,k2+1) < 0 )
        v_m5(i,k2+1) = 0;
    end

end

% flow link M5

for i=2:N_m(5,1)+1
    q_m5(i,k2+1) = rho_m5(i-1,k2+1) * v_m5(i,k2+1);
end

%    % accelerazioni segmental
% 
%    for i=2:N_m(5,1)+1
%        a_seg_m5(i-1,k) = ( ( v_m5(i,k+1) - v_m5(i,k) ) / T ) * conv_fact;
%    end
% 
%  % accelerazioni cross segmental
% 
%  for i=2:N_m(5,1)+1
%      if (i==N_m(5,1)+1)
%          a_cross_m5_m10(i-1,k) = ( ( v_m10(1,k+1) - v_m5(i,k) ) / T ) * conv_fact;
%      else
%          a_cross_m5(i-1,k) = ( ( v_m5(i+1,k+1) - v_m5(i,k) ) / T ) * conv_fact;
%      end
%  end
% 
%   % numero di veicoli
% 
%     % n segmental
%     for i=2:N_m(5,1)+1
%         n_seg_m5(i-1,k) = Delta_M(5,1) * rho_m5(i,k) - T * q_m5(i,k);
%     end
% 
%     % n cross segmental
%     for i=2:N_m(5,1)+1
%         if (i==N_m(5,1)+1)
%             n_cross_m5_m10(i,k) = T * q_m10(1,k);
%         else
%             n_cross_m5(i-1,k) = T * q_m5(i,k);
%         end
%     end

       %% on-ramp link model %%%
    
       % partial demand
       for j=1:J_r(4,1) % Destination reachable from o4
           partial_demand_o4(j,k2) = demand_o4(1,k2) * theta_o4(j,k2);
       end

       l_o4(1,k2) = partial_l_o4(1,k2);

       q_o4(1,k2) = min( [ demand_o4(1,k2) + l_o4(1,k2) / T ; r_1_cap ;...
           r_1_cap * ( ( rho_max - rho_m10(1,k2)) / (rho_max - rho_cr ) ) ]);


       for j=1:J_r(4,1)
           if( partial_l_o4(j,k2) == 0 && l_o4(1,k2)== 0)
               gamma_o4(j,k2) = theta_o4(j,k2);
           else
               gamma_o4(j,k2) = partial_l_o4(j,k2) / l_o4(1,k2);
           end
       end
    
    
      % partial on-ramp queue o4
      for j=1:J_r(4,1)
          partial_l_o4(j,k2+1) = partial_l_o4(j,k2) + T * ( demand_o4(1,k2) * theta_o4(j,k2) - gamma_o4(j,k2) * q_o4(1,k2) );

          if(partial_l_o4(j,k2+1) <0)
              partial_l_o4(j,k2+1) =0;
          end
      end

    
    
%       % acc sulla rampa
%     
%       % acc arrival vehicles
%       a_arr_o4(1,k) = ( ( v_idl_1(1,k+1) - v_on_1(1,k) ) / T ) * conv_fact;
%       % acc waiting vehicles
%       a_w_o4(1,k) = ( ( v_idl_1(1,k+1) - v_idl_1(1,k) ) / T ) * conv_fact;
%       % acc leaving with stops vehicles
%       a_ls_o4(1,k) = ( ( v_m10(1,k+1) - v_idl_1(1,k) ) / T ) * conv_fact;
%       % acc leaving without stops vehicles
%       a_lns_o4(1,k) = ( ( v_m10(1,k+1) - v_on_1(1,k) ) / T ) * conv_fact;
%     
%     % numero veh auto
%     if ( q_o4(1,k) >= 0 ) && ( q_o4(1,k) <= (l_o4(1,k) / T) ) % scenario 1 auto
% 
%         %n arrival vehicles
%         n_arr_o4(1,k) = T * demand_o4(1,k);
% 
%         % n waiting vehicles
%         n_w_o4(1,k) = l_o4(1,k) - T * q_o4(1,k);
% 
%         % n leaving with stops vehicles
%         n_ls_o4(1,k) = T * q_o4(1,k);
% 
%         % n leaving without stops vehicles
%         n_lns_o4(1,k) = 0;
% 
%     elseif ( q_o4(1,k) <= (l_o4(1,k) / T + demand_o4(1,k) )) && ( q_o4(1,k) > ( l_o4(1,k) / T ) ) %scenario 2 auto
% 
%         % n arrival vehicles
%         n_arr_o4(1,k) = T * demand_o4(1,k) + l_o4(1,k) - T * q_o4(1,k);
% 
%         % n waiting vehicles
%         n_w_o4(1,k) = 0;
% 
%         % n leaving with stops vehicles
%         n_ls_o4(1,k) = l_o4(1,k);
% 
%         % n leaving without stops vekicles
%         n_lns_o4(1,k) = T * q_o4(1,k) - l_o4(1,k);
%     end
%     
    %
    % node model %%%

    % flows entering from the nodes N6

    Q_n6(1,k2) = q_m5(N_m(5,1)+1,k2) * gamma_m5(1,N_m(5,1)+1,k2) + q_o4(1,k2) * gamma_o4(1,k2);  % Node N6 destination D1 class 1

    % outgoing traffic flow N6

    q_m10(1,k2) = beta_m10(1,k2) * Q_n6(1,k2);

    %%% freeway link model %%%

    % gamma M10

    for j=1:J_m(10,1) %alla gamma della sezione zero gli assegno il valore di quella della sezione 1
        if ( partial_rho_m10(j,1,k2) == 0 && rho_m10(1,k2) == 0 )
            gamma_m10(j,1,k2) = 0;
        else
            gamma_m10(j,1,k2) = partial_rho_m10(j,1,k2) / rho_m10(1,k2);
        end
    end

    for j=1:J_m(10,1)
        for i=2:N_m(10,1)+1
            if ( partial_rho_m10(j,i-1,k2) == 0 && rho_m10(i-1,k2) == 0 )
                gamma_m10(j,i,k2) = 0;
            else
                gamma_m10(j,i,k2) = partial_rho_m10(j,i-1,k2) / rho_m10(i-1,k2);
            end
        end
    end

    % partial density link M10

    for j=1:J_m(10,1)  % for each destination
        for i=1:N_m(10,1) %for each section
            partial_rho_m10(j,i,k2+1) = partial_rho_m10(j,i,k2) + T / Delta_M(10,1) * ( gamma_m10(j,i,k2) * q_m10(i,k2) ...
                - gamma_m10(j,i+1,k2) * q_m10(i+1,k2) );
        end
    end

    % density link M10

    rho_m10(N_m(10,1)+1,k2) =  rho_finD1; %virtual density i+1

    for i=1:N_m(10,1)
        rho_m10(i,k2+1) = partial_rho_m10(1,i,k2+1);
    end

    if ( rho_m10(1,k2) == 0 && rho_m10(1,k2) == 0)
        rho_m10(N_m(10,1)+1,k2) = 0;
    end


    %  mean speed link M10

    v_m10(1,k2) =  v_m5(N_m(5,1),k2);

    for i=2:N_m(10,1)+1 % i=1 consider the virtual speed in i-1

        v_m10(i,k2+1) = v_m10(i,k2) + ( T / tau_1 ) * ( vf_1 * ( 1 - ( ( rho_m10(i-1,k2) ) / rho_max )^ l1 )^m1 - v_m10(i,k2) )...
            +( T/Delta_M(10,1) ) * v_m10(i,k2) * ( v_m10(i-1,k2) - v_m10(i,k2) ) - ( ( nu_1 * T * ( rho_m10(i,k2) ...
            - rho_m10(i-1,k2)  ) ) / ( Delta_M(10,1) * tau_1 * ( rho_m10(i-1,k2) + chi_1 ) ) );


        if ( v_m10(i,k2+1) < 0 )
            v_m10(i,k2+1) = 0;
        end


    end

    % flow link M10

    for i=2:N_m(10,1)+1
        q_m10(i,k2+1) = rho_m10(i-1,k2+1) * v_m10(i,k2+1);
    end


%     % accelerazioni segmental
% 
%     for i=2:N_m(10,1)+1
%         a_seg_m10(i-1,k) = ( ( v_m10(i,k+1) - v_m10(i,k) ) / T ) * conv_fact;
%     end
% 
%     % accelerazioni cross segmental
% 
%     for i=2:N_m(10,1)+1
%         if (i==N_m(10,1)+1)
%             a_cross_m10(i-1,k) = ( ( v_m10(i,k+1) - v_m10(i-1,k) ) / T ) * conv_fact;
%         else
%             a_cross_m10(i-1,k) = ( ( v_m10(i+1,k+1) - v_m10(i,k) ) / T ) * conv_fact;
%         end
%     end
% 
%     % numero di veicoli
% 
%     % n segmental
%     for i=2:N_m(10,1)+1
%         n_seg_m10(i-1,k) = Delta_M(10,1) * rho_m10(i,k) - T * q_m10(i,k);
%     end
% 
%     % n cross segmental
%     for i=2:N_m(10,1)+1
%         n_cross_m10(i-1,k) = T * q_m10(i,k);
%     end

    q_d1(1,k2) = q_m10(N_m(10,1)+1,k2);



    %% on-ramp link model %%%

    % partial demand
    for j=1:J_r(5,1) % Destination reachable from o6
        partial_demand_o6(j,k2) = demand_o6(1,k2) * theta_o6(j,k2); % cars
    end

    l_o6(1,k2) = partial_l_o6(1,k2);
    q_o6(1,k2) = min( [ demand_o6(1,k2) + l_o6(1,k2) / T ; r_1_cap ;...
        r_1_cap * ( ( rho_max - rho_m11(1,k2)  ) / (rho_max - rho_cr ) ) ]);


    for j=1:J_r(5,1)
        if( partial_l_o6(j,k2) == 0 && l_o6(1,k2)== 0)
            gamma_o6(j,k2) = theta_o6(j,k2);
        else
            gamma_o6(j,k2) = partial_l_o6(j,k2) / l_o6(1,k2);
        end
    end


    % partial on-ramp queue o6
    for j=1:J_r(5,1)
        partial_l_o6(j,k2+1) = partial_l_o6(j,k2) + T * ( demand_o6(1,k2) * theta_o6(j,k2) - gamma_o6(j,k2) * q_o6(1,k2) );

        if(partial_l_o6(j,k2+1) <0)
            partial_l_o6(j,k2+1) =0;
        end
    end



%     % acc sulla rampa
% 
%     % acc arrival vehicles
%     a_arr_o6(1,k) = ( ( v_idl_1(1,k+1) - v_on_1(1,k) ) / T ) * conv_fact;
% 
%     % acc waiting vehicles
%     a_w_o6(1,k) = ( ( v_idl_1(1,k+1) - v_idl_1(1,k) ) / T ) * conv_fact;
% 
%     % acc leaving with stops vehicles
%     a_ls_o6(1,k) = ( ( v_m11(1,k+1) - v_idl_1(1,k) ) / T ) * conv_fact;
% 
%     % acc leaving without stops vehicles
%     a_lns_o6(1,k) = ( ( v_m11(1,k+1) - v_on_1(1,k) ) / T ) * conv_fact;
% 
%     % numero veh auto
%     if ( q_o6(1,k) >= 0 ) && ( q_o6(1,k) <= (l_o6(1,k) / T) ) % scenario 1 auto
% 
%         %n arrival vehicles
%         n_arr_o6(1,k) = T * demand_o6(1,k);
% 
%         % n waiting vehicles
%         n_w_o6(1,k) = l_o6(1,k) - T * q_o6(1,k);
% 
%         % n leaving with stops vehicles
%         n_ls_o6(1,k) = T * q_o6(1,k);
% 
%         % n leaving without stops vehicles
%         n_lns_o6(1,k) = 0;
% 
%     elseif ( q_o6(1,k) <= (l_o6(1,k) / T + demand_o6(1,k) )) && ( q_o6(1,k) > ( l_o6(1,k) / T ) ) %scenario 2 auto
% 
%         % n arrival vehicles
%         n_arr_o6(1,k) = T * demand_o6(1,k) + l_o6(1,k) - T * q_o6(1,k);
% 
%         % n waiting vehicles
%         n_w_o6(1,k) = 0;
% 
%         % n leaving with stops vehicles
%         n_ls_o6(1,k) = l_o6(1,k);
% 
%         % n leaving without stops vekicles
%         n_lns_o6(1,k) = T * q_o6(1,k) - l_o6(1,k);
%     end


% flows entering from the nodes N9
Q_1_n9(1,k2) = q_m9(N_m(9,1)+1,k2) * gamma_m9(1,N_m(9,1)+1,k2) + q_o6(1,k2) * gamma_o6(1,k2); % Node N6 destination D2 class 1

% outgoing traffic flow N9
q_m11(1,k2) = beta_m11(1,k2) * Q_1_n9(1,k2); %flusso diretto a D2 che utilizzano il link m11

%%% freeway link model %%%

% gamma M11

for j=1:J_m(11,1) %alla gamma della sezione zero gli assegno il valore di quella della sezione 1
    if ( partial_rho_m11(j,1,k2) == 0 && rho_m11(1,k2) == 0 )
        gamma_m11(j,1,k2) = 0;
    else
        gamma_m11(j,1,k2) = partial_rho_m11(j,1,k2) / rho_m11(1,k2);
    end
end

for j=1:J_m(11,1)
    for i=2:N_m(11,1)+1
        if ( partial_rho_m11(j,i-1,k2) == 0 && rho_m11(i-1,k2) == 0 )
            gamma_m11(j,i,k2) = 0;
        else
            gamma_m11(j,i,k2) = partial_rho_m11(j,i-1,k2) / rho_m11(i-1,k2);
        end
    end
end

% partial density link M11

for j=1:J_m(11,1)  % for each destination
    for i=1:N_m(11,1) %for each section
        partial_rho_m11(j,i,k2+1) = partial_rho_m11(j,i,k2) + T / Delta_M(11,1) * ( gamma_m11(j,i,k2) * q_m11(i,k2) ...
            - gamma_m11(j,i+1,k2) * q_m11(i+1,k2) );
    end
end

% density link M11

rho_m11(N_m(11,1)+1,k2) =  rho_finD2; %virtual density i+1

for i=1:N_m(11,1)
    rho_m11(i,k2+1) = partial_rho_m11(1,i,k2+1);
end

if ( rho_m11(1,k2) == 0 && rho_m11(1,k2) == 0)
    rho_m11(N_m(11,1)+1,k2) = 0;
end



%  mean speed link M11

v_m11(1,k2) = v_m9(N_m(9,1)+1,k2) ;

for i=2:N_m(11,1)+1 % i=1 consider the virtual speed in i-1



    v_m11(i,k2+1) = v_m11(i,k2) + ( T / tau_1 ) * ( vf_1 * ( 1 - ( ( rho_m11(i-1,k2) ) / rho_max )^ l1 )^m1 - v_m11(i,k2) )...
        +( T/Delta_M(11,1) ) * v_m11(i,k2) * ( v_m11(i-1,k2) - v_m11(i,k2) ) - ( ( nu_1 * T * ( rho_m11(i,k2) ...
        - rho_m11(i-1,k2) ) ) / ( Delta_M(11,1) * tau_1 * ( rho_m11(i-1,k2) + chi_1 ) ) );

    if ( v_m11(i,k2+1) < 0 )
        v_m11(i,k2+1) = 0;
    end

end

% flow link M11

for i=2:N_m(11,1)+1
    q_m11(i,k2+1) = rho_m11(i-1,k2+1) * v_m11(i,k2+1);
end
    
% % accelerazioni segmental
% 
% for i=2:N_m(11,1)+1
%     a_seg_m11(i-1,k) = ( ( v_m11(i,k+1) - v_m11(i,k) ) / T ) * conv_fact;
% end
% 
% % accelerazioni cross segmental
% 
% for i=2:N_m(11,1)+1
%     if (i==N_m(11,1)+1)
%         a_cross_m11(i-1,k) = ( ( v_m11(i,k+1) - v_m11(i-1,k) ) / T ) * conv_fact;
%     else
%         a_cross_m11(i-1,k) = ( ( v_m11(i+1,k+1) - v_m11(i,k) ) / T ) * conv_fact;
%     end
% end
% 
% % numero di veicoli
% 
% % n segmental
% for i=2:N_m(11,1)+1
%     n_seg_m11(i-1,k) = Delta_M(11,1) * rho_m11(i,k) - T * q_m11(i,k);
% end
% 
% % n cross segmental
% for i=2:N_m(11,1)+1
%     n_cross_m11(i-1,k) = T * q_m11(i,k);
% end

q_d2_nc(1,k2) = q_m11(N_m(11,1)+1,k2);




end