classdef Vehicle_Following < NewManeuver
    %VehicleFollowing Summary of this class goes here
    %   Detailed explanation goes here
    
    methods
        
        function decisions = getDecisionsForDrivingMode(obj, state, d_goal, accAll, name_DrivingMode,ManeuverPlanner,states_Other)
            
            % Get the decisions for a driving mode according to the MPC acceleration sequence prediction
            MPC_States = Vehicle_Following.findClosestVehicleOnSameLane(state,states_Other);
            
            if isempty(MPC_States)
                decisions = []; % No prediction if no other vehicle around
                return
            end
            
            acc_sequence = Vehicle_Following.MPC_Predict(MPC_States);
            
            %futureTrajectory = Vehicle_Following.calculateFutureState(state,acc_sequence);
            
            
            
            futureState = state;
            wholetrajectoryFrenet =[];
            
            
            
            
            for acc=acc_sequence'
                
                timeHorizon = ManeuverPlanner.Th/length(acc_sequence); % Constant acc horizon motion time
                
                
                trajectoryFrenet = ...
                    Vehicle_Following.calculateLongitudinalTrajectory(ManeuverPlanner.Ts,futureState.s, d_goal, ...
                    futureState.speed, ...
                    ManeuverPlanner.vEgo_ref, acc, ...
                    timeHorizon);
                
                % To fit the format (TODO: rework later)
                trajectoryFrenet.s(1) = [];
                trajectoryFrenet.d(1) = [];
                trajectoryFrenet.velocity(1) = [];
                trajectoryFrenet.time(1) = [];
                trajectoryFrenet.length = trajectoryFrenet.length -1;

                futureState = State(trajectoryFrenet.s(end), trajectoryFrenet.d(end), ...
                    0, trajectoryFrenet.velocity(end));
                
                wholetrajectoryFrenet = [wholetrajectoryFrenet trajectoryFrenet];
            end
            
            % Predicted trajectory
            trajectoryFrenet = FrenetTrajectory((0:ManeuverPlanner.Ts:ManeuverPlanner.Th),...
                vertcat(state.s,wholetrajectoryFrenet.s), vertcat(state.d,wholetrajectoryFrenet.d)...
                , vertcat(state.speed,wholetrajectoryFrenet.velocity), true, 0);
            
            description = [name_DrivingMode, '_{acc', num2str(acc_sequence(1)), '}'];
            
            % Future state prediction
            [~, futureOrientation] = Frenet2Cartesian(trajectoryFrenet.s(end), ...
                trajectoryFrenet.d(end), ...
                ManeuverPlanner.RoadTrajectory);
            futureState = State(trajectoryFrenet.s(end), trajectoryFrenet.d(end), ...
                futureOrientation, trajectoryFrenet.velocity(end));
            
            % Discrete trajectory
            trajectoryDiscrete = Continuous2Discrete(ManeuverPlanner.spaceDiscretisation, trajectoryFrenet);
            
            new_Maneuver = NewManeuver(obj.name,obj.id,obj.NewTrajectoryGenerator);
            
            decisions = new_Maneuver.assignTrajectory(trajectoryDiscrete, true, futureState, description, [], []);
            
        end
        
        
        
        
        
    end
    
    
    
    methods (Static)
        function nextState = apply(state,deltaT)
            %Apply VehicleFollowing Maneuver
            speed_new = state.speed;
            s_new = state.s + speed_new*deltaT;
            d_new = state.d;
            orientation_new = state.orientation;
            
            nextState = State(s_new,d_new,orientation_new,speed_new);
        end
        
        
        function MPC_States = findClosestVehicleOnSameLane(state,states_Other)
            % Checks for the closest vehicle ahead and sets up the state
            % vector for the MPC as [relDist relSpeed EgoSpeed]; if no other vehicle then return empty
            
            if isempty(states_Other)
                MPC_States = []; % No prediction if no other vehicle around
                return
            end
            
            % Get only the vehicles on the same lane
            vehiclesOnTheSameLane = states_Other([states_Other.d]==state.d);
            
            if isempty(vehiclesOnTheSameLane)
                MPC_States = [];
                return
            end
            
            % Get only the vehicles ahead on the same lane
            vehiclesAheadOnTheSameLane = vehiclesOnTheSameLane([vehiclesOnTheSameLane.s]>state.s);
            if isempty(vehiclesAheadOnTheSameLane)
                MPC_States = [];
                return
            end
            
            % Get only the closest vehicle ahead on the same lane
            [closestRelDistance, idx] = min([vehiclesAheadOnTheSameLane.s]);
            
            relativeSpeed = vehiclesAheadOnTheSameLane(idx).speed - state.speed;
            MPC_States = [closestRelDistance relativeSpeed state.speed];
        end
        
        
        function acc_sequence = MPC_Predict(States)
            %% output of S-Function
            % to do:put a equation number
            % here define the system matrices, estimate the state and optimize the
            % quadratic programming.
            x_k = [0;0;0];
            U = 0;
            % sample time
            Ts = 0.5;
            %% discrete state-space model: A, B, C, S, Z
            % state space model: x(k+1|k)=A*x(k)+B*u(k)+S*d(k), p5, (2.16)
            % x(k)=[xr(k);vh(k);vr(k)]
            % xr: relative distance, vr: relative speed, vh: ego vehicle speed
            % system matrix, p5, (2.18)
            A = [1 Ts 0;
                0 1  0;
                0 0  1];
            % input matrix, p6, (2.19)
            B = [-0.5*Ts^2; -Ts; Ts];
            % disturbance matrix, p6, (2.20)
            S = [0.5*Ts^2; Ts; 0];
            % y(k)=[e(k); vr(k)]
            % spacing error: e(k) = xr(k) - xr,des(k), p5, (2.12)
            % desired following distance:xr,des(k) = xr0 + vh(k)*t_hw, p5, (2.11)
            % xr0:fixed safety distance
            % desired headway time
            t_hw = 1.4;
            fixed_safety_distance = 10;
            % y(k) = C*x(k)-Z
            % observation matrix, p6, (2.21)
            C = [1 0 -t_hw;
                0 1  0   ];
            % p6, (2.22)
            Z = [fixed_safety_distance;0];
            %% output error vector: A_bar, B_bar, S_bar, Z_bar
            % Y = A_bar*x(k)+B_bar*U+S_bar*D-Z_bar, p10, (3.24)
            % Y = [y(k+1|k)-yr(k+1|k),...,y(k+Np|k)-yr(k+Np|k)]
            % yr(k+i|k)=[0; 0]
            % note: dim_A_1: size(A, 1); dim_A_2: size(A, 2)
            % A_bar = [C*A; ...; C*A^Np], p10, (3.25)
            % size: Np*dim_C_1 x dim_A_2
            % prediction horizon
            Np = 10;
            A_bar = cell(Np, 1);
            A_bar{1, 1} = C*A;
            for i=2:Np
                A_bar{i, 1} = A_bar{i-1, 1}*A;
            end
            A_bar = cell2mat(A_bar);
            % B_bar = [C*B            0               0                ... 0;
            %          C*A*B          C*B             0                ... 0;
            %          ...
            %          C*A^(Np-1)*B   C*A^(Np-2)*B    C*A^(Np-3)*B     ... C*B]
            % p11, (3.26), size: Np*dim_C_1 x Np*dim_B_2
            n_C = size(C, 1);
            B_bar = cell(Np, Np);
            for i=1:Np
                for j=1:Np
                    if j>i
                        % upper triangular values are 0
                        B_bar{i, j} = zeros(n_C, 1);
                    else
                        % only assigne lower triangular values
                        B_bar{i, j} = C*A^(i-j)*B;
                    end
                end
            end
            B_bar = cell2mat(B_bar);
            % S_bar = [C*S            0               0                ... 0;
            %          C*A*S          C*S             0                ... 0;
            %          ...
            %          C*A^(Np-1)*S   C*A^(Np-2)*S    C*A^(Np-3)*S     ... C*S]
            % p11, (3.27), size: Np*dim_C_1 x Np*dim_S_2
            S_bar = cell(Np, Np);
            for i=1:Np
                for j=1:Np
                    if j>i
                        % upper triangular values are 0
                        S_bar{i, j} = zeros(n_C, 1);
                    else
                        % only assigne lower triangular values
                        S_bar{i, j} = C*A^(i-j)*S;
                    end
                end
            end
            S_bar = cell2mat(S_bar);
            % Z_bar = [Z_1; ...; Z_Np], p11, (3.29)
            % size: dim_Z_1*Np x dim_Z_2
            Z_bar = cell(Np, 1);
            for i=1:Np
                Z_bar{i, 1} = Z;
            end
            Z_bar = cell2mat(Z_bar);
            %% cost: Q_bar, R_bar
            % cost function, p7, (3.1):
            % J = ∑(i=1 to Np)(y(k+i|k)-yr(k+i|k))'*Q*(y(k+i|k)-yr(k+i|k)) +
            %     ∑(i=0 to Np-1)(u(k+i|k))'*R*(u(k+i|k)) +
            %     ∑(i=0 to Np-2)(u(k+i+1|k)-u(k+i|k))'*Ru*(u(k+i+1|k)-u(k+i|k))
            % matrix form: J = Y'*Q_bar*Y + U'*R_bar*U + U'*G'Ru_bar*G*U, p10, (3.23)
            % U=[u(k|k);...;u(k+Np-1|k)],G=[-1 1 0 ... 0;0 -1 1 0 ... 0;...;0 ... -1 1]
            %Optional:
            % Q = [15  0;
            %      0 7];
            % Q = [0.5 0;
            %      0   7];
            Q = [10 0;
                0  7];
            % optional: R = 20;
            R = 50;
            % Q_bar = diag(Q), Q: weight matrix for output error part, p9, (3.11)
            % size: dim_Q_1*Np x Np*dim_Q_2
            Q_bar = cell(Np, Np);
            % R_bar = diag(R), R: weight matrix for control input part, p9, (3.14)
            % size: dim_R_1*Np x Np*dim_R_2
            R_bar = cell(Np, Np);
            n_R = size(R, 1);
            for i=1:Np
                for j=1:Np
                    if i==j
                        % only assigned diagonal values
                        Q_bar{i, i} = Q;
                        R_bar{i, i} = R;
                    else
                        % otherwises 0
                        Q_bar{i, j} = zeros(n_C, n_C);
                        R_bar{i, j} = zeros(n_R, n_R);
                    end
                end
            end
            Q_bar = cell2mat(Q_bar);
            R_bar = cell2mat(R_bar);
            %% cost: G, Ru and Ru_bar
            % G=1/Ts*[-1  1 0 0 ... 0  0;
            %         0 -1 1 0 ... 0  0;
            %         ...
            %         0 ...0 0 ... -1 1]
            % p10, (3.19)
            % becuase of the differentiation, the size of G is Np-1 x Np
            G = zeros(Np-1, Np);
            for i=1:Np-1
                G(i, i) = -1;
                G(i, i+1) = 1;
            end
            G = 1/Ts*G;
            % Ru_bar = diag(Ru), p10, (3.20)
            % size: dim_Ru_1*(Np-1) x (Np-1)*dim_Ru_2
            % Ru, weight for u(k+i+1|k)-u(k+i|k) part, or jerk part in cost function
            Ru = 70;
            Ru_bar = cell(Np-1, Np-1);
            for i=1:Np-1
                for j=1:Np-1
                    if i==j
                        % only assign diagonal values
                        Ru_bar{i, j} = Ru;
                    else
                        % otherwise 0
                        Ru_bar{i, j} = 0;
                    end
                end
            end
            Ru_bar = cell2mat(Ru_bar);
            %% measure and relative speed at last time step
            % Current measured state: x_k=[xr(k);vr(k);vh(k)]
            xr = States(1);
            vr = States(2);
            vh = States(3);
            
            x_mess = [xr;vr;vh];
            ve_last_k = x_k(2);
            
            % without Kalman
            x_k = x_mess;
            
            %% cost: f^T, H, f_bar^T, H_bar
            % J = f_T*U + U'*H*U + U'*G'*R_u_bar*G*U + p*ep^2, p19, (4.39)
            % U=[u(k|k);...;u(k+Np-1|k)], p9, (3.13); ep: slack variable
            % leader vehicle acceleration as disturbance and is assumed to be constant
            % in the next Np time step. p11, (3.30), (3.31)
            ve_k = x_k(2);
            d = (ve_k - ve_last_k)/Ts + U;
            D = d*ones(Np, 1);
            disturb = D'*S_bar'*Q_bar*B_bar;
            % f^T, p12, (3.46)
            f_T = 2*(x_k'*A_bar'*Q_bar*B_bar-Z_bar'*Q_bar*B_bar+disturb);
            % f_bar^T, p19, (4.42)
            f_bar_T = [f_T 0]';
            % H, p12, (3.45)
            H = 2*(B_bar'*Q_bar*B_bar + R_bar + G'*Ru_bar*G);
            % p: weight for slack variable, optional: p = 10
            p = 1e-2;
            % H_bar, p19, (4.41)
            H_bar= [H zeros(Np,1);
                zeros(1, Np) p];
            %% Constraints: L, M, N
            % [xr(k);vh(k)] = L*x(k), L = [1 0 0;0 0 1], p15, (4.7)
            L = [1 0 0;
                0 0 1];
            % M=[minimal_relative_distance;vmin]<=L*x(k+1)
            % = L*A*x(k)+L*B*u(k)+L*S*d(k), p15, (4.11)
            % -L*B*u(k) <= -M + L*A*x(k) + L*S*d(k), p16, (4.12)
            % maximal and minimal of ego vehicle speed
            vmax = 20;
            vmin = 0;
            minimal_relative_distance = 10;
            M = [minimal_relative_distance;vmin];
            % N = [maximal_relative_distance;vmax] >= L*x(k+1), p15, (4.11)
            %  L*B*u(k) <=  N - L*A*x(k) - L*S*d(k), p16, (4.12)
            N = [10000;vmax];
            % delta_U = [u(k+1)-u(k);...;u(k+Np-1)-u(k+Np-2)] = GU, p10, (3.18),(3.21)
            % GU <= Jmax, p18, (4.28)
            % GU >= Jmin, p18, (4.29)
            % maximal and minimal of control input acceleration
            amax = 2;
            amin = -3;
            %% Constraints: L_bar, Ae_bar, Be_bar, Se_bar
            % L_bar = diag(L), p17, (4.19), size: Np*dim_L_1 x dim_L_2*Np
            n_L1 = size(L, 1);
            n_L2 = size(L, 2);
            L_bar = cell(Np, Np);
            for i=1:Np
                for j=1:Np
                    if i==j
                        % assign only diagonal values
                        L_bar{i, j} = L;
                    else
                        % otherwise 0
                        L_bar{i, j} = zeros(n_L1, n_L2);
                    end
                end
            end
            L_bar = cell2mat(L_bar);
            % Ae_bar = [A;...;A^Np], size: Np*dim_A_1 x dim_A_2, p17, (4.20)
            Ae_bar = cell(Np, 1);
            Ae_bar{1, 1} = A;
            for i=2:Np
                Ae_bar{i, 1} = Ae_bar{i-1, 1}*A;
            end
            Ae_bar = cell2mat(Ae_bar);
            % Be_bar = [B           0            0         ... 0;
            %           A*B         B            0         ... 0;
            %           ...
            %           A^(Np-1)*B  A^(Np-2)*B   A^(Np-3)*B  ... B]
            % size: Np*dim_B_1 x dim_B_2xNp, p17, (4.21)
            n_B = size(B, 1);
            Be_bar = cell(Np, Np);
            for i=1:Np
                for j=1:Np
                    if j>i
                        Be_bar{i, j} = zeros(n_B, 1);
                    else
                        Be_bar{i, j} = A^(i-j)*B;
                    end
                end
            end
            Be_bar = cell2mat(Be_bar);
            % Se_bar = [S           0            0         ... 0;
            %           A*S         S            0         ... 0;
            %           ...
            %           A^(Np-1)*S  A^(Np-2)*S   A^(Np-3)*S  ... S]
            % size: Np*dim_S_1 x dim_S_2xNp, p17, (4.22)
            Se_bar = cell(Np, Np);
            n_S = size(S, 1);
            for i=1:Np
                for j=1:Np
                    if j>i
                        % upper triangular values are 0
                        Se_bar{i, j} = zeros(n_S, 1);
                    else
                        % only assign lower triangular values
                        Se_bar{i, j} = A^(i-j)*S;
                    end
                end
            end
            Se_bar = cell2mat(Se_bar);
            
            %% Constraints: Omega_bar, T
            % Omega*U <= T, p18, (4.30)
            % Omega = [L_bar*Be_bar;-L_bar*Be_bar;G;-G], p18, (4.31)
            % with slack variable, Omega_bar*U_bar <= T, p20, (4.50)
            % Omega_bar, p20, (4.51)
            % T = [N_bar-L_bar*Ae_bar*xe(k)-L_bar*Se_bar*D;
            %     -M_bar+L_bar*Ae_bar*xe(k)+L_bar*Se_bar*D;
            %     Jmax;
            %     Jmin], p19,(4.34)/p20, (4.52)
            
            n_N = size(N, 1);
            Omega_bar = cell(4, 1);
            Omega_bar{1, 1} = [L_bar*Be_bar -ones(n_N*Np, 1)];
            Omega_bar{2, 1} = [-L_bar*Be_bar zeros(n_N*Np, 1)];
            Omega_bar{3, 1} = [G -ones(Np-1, 1)];
            Omega_bar{4, 1} = [-G ones(Np-1, 1)];
            Omega_bar = cell2mat(Omega_bar);
            N_bar = cell(Np, 1);
            M_bar = cell(Np, 1);
            for i=1:Np
                N_bar{i, 1} = N;
                M_bar{i, 1} = M;
            end
            N_bar = cell2mat(N_bar);
            M_bar = cell2mat(M_bar);
            % T
            % lower bound, upper bound of jerk
            jmax = 2.5;
            jmin = -2.5;
            T = cell(4, 1);
            T{1, 1} =  N_bar - L_bar*Ae_bar*x_k - L_bar*Se_bar*D;
            T{2, 1} = -M_bar + L_bar*Ae_bar*x_k + L_bar*Se_bar*D;
            T{3, 1} = jmax*ones(Np-1, 1);
            T{4, 1} = -jmin*ones(Np-1, 1);
            T = cell2mat(T);
            %% optimization
            options = optimset('Algorithm','active-set');
            options.Display = 'none';
            options.Diagnostics = 'off';
            % Inequality constraint
            A_ine = Omega_bar;
            b_ine = T;
            % No equality constraint, since the state-space equation (2.16), (2,17) is
            % already incorperated in the derivation of (3.38)
            A_ep = [];
            B_ep=[];
            % lower bound, upper bound.
            Umax = amax*ones(Np, 1);
            Umin = amin*ones(Np, 1);
            ep_min = 0;
            ep_max = 100;
            lower_bound = [Umin;ep_min];
            upper_bound = [Umax;ep_max];
            % initial point is chosen to zero because of convex problem, we can
            % always find a global minimum, and it might be faster when we choose another
            % initial point
            ini = zeros(Np+1,1);
            fun = @(U)U'*H_bar*U+f_bar_T'*U;
            warning off
            options = optimset();
            options.Display = 'none';
            options.Diagnostics = 'off';
            acc_sequence = fmincon(fun, ini, A_ine,b_ine,A_ep,B_ep,lower_bound,upper_bound,[],options);
            warning on
            acc_sequence = acc_sequence(1:end-1); % Remove the slack variable from the output just leave acc sequence
        end
        
        function predictedState = calculateFutureState(state,acc_sequence)
            % Calculate the predicted positions and speeds at the changing
            % acceleration points (sequence) obtained from the MPC
            Ts = 0.5;
            ini= [state.s state.speed];
            
            A = [1 Ts; 0 1];
            B = [(Ts^2)/2; Ts];
            
            A_i = [A;A^2;A^3;A^4;A^5;A^6;A^7; A^8; A^9; A^10];
            
            
            B_i = [B zeros(2,1) zeros(2,1) zeros(2,1) zeros(2,1) zeros(2,1) zeros(2,1) zeros(2,1) zeros(2,1) zeros(2,1);...
                A*B B  zeros(2,1) zeros(2,1) zeros(2,1) zeros(2,1) zeros(2,1) zeros(2,1) zeros(2,1) zeros(2,1);...
                (A^2)*B A*B B zeros(2,1) zeros(2,1) zeros(2,1) zeros(2,1) zeros(2,1) zeros(2,1) zeros(2,1);...
                (A^3)*B (A^2)*B A*B B zeros(2,1) zeros(2,1) zeros(2,1) zeros(2,1) zeros(2,1) zeros(2,1);...
                (A^4)*B (A^3)*B (A^2)*B A*B B zeros(2,1) zeros(2,1) zeros(2,1) zeros(2,1) zeros(2,1);...
                (A^5)*B (A^4)*B (A^3)*B (A^2)*B A*B B zeros(2,1) zeros(2,1) zeros(2,1) zeros(2,1);...
                (A^6)*B (A^5)*B (A^4)*B (A^3)*B (A^2)*B A*B B zeros(2,1) zeros(2,1) zeros(2,1);...
                (A^7)*B (A^6)*B (A^5)*B (A^4)*B (A^3)*B (A^2)*B A*B B zeros(2,1) zeros(2,1);...
                (A^8)*B (A^7)*B (A^6)*B (A^5)*B (A^4)*B (A^3)*B (A^2)*B A*B B zeros(2,1); ...
                (A^9)*B (A^8)*B (A^7)*B (A^6)*B (A^5)*B (A^4)*B (A^3)*B (A^2)*B A*B B]; ...
                
            predictedState = A_i * ini' + B_i * acc_sequence(1:10);
            predictedState = [ini'; predictedState];
            
            
        end
        
        function trajectoryFrenet = calculateLongitudinalTrajectory(Ts,s_0, d_0, v_0, v_max, ...
                acceleration, duration)
            % Calculate longitudinal trajectory according to the double integrator model
            
            time = (0:Ts:duration)'; % Start with 0 (relative time)
            [s_trajectory, v_trajectory] = Vehicle_Following.predictMotion(s_0, v_0, v_max, acceleration, time);
            d_trajectory = d_0*ones(length(time), 1);
            
            trajectoryFrenet = FrenetTrajectory(time, s_trajectory, d_trajectory, v_trajectory, ...
                true, 0);
        end
        
        function [s, v] = predictMotion(s_0, v_0, v_limit, a, time)
            % Restricted equation of motion for displacement and velocity according to double integrator
            
            s = s_0 + v_0*time + 0.5*a*time.^2;
            v = v_0 + a*time;
            if any(v > v_limit) % Not faster than limit velocity
                id_limit = find(v>v_limit, 1);
                s(v>v_limit) = s(id_limit) + v_limit*(time(v>v_limit) - time(id_limit));
                v(v>v_limit) = v_limit;
            end
            if any(v < 0) % No backward motion
                id_limit = find(v<0, 1);
                s(v<0) = s(id_limit);
                v(v<0) = 0;
            end
        end
        
        
        
        
    end
end

