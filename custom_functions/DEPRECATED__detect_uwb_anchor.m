classdef detect_uwb_anchor < matlab.System
    % Untitled2 Add summary here
    %
    % This template includes the minimum set of functions required
    % to define a System object with discrete state.

    % Public, tunable properties
    properties
        % Process noise
        w_p = 1e-5
        
        % Measurement noise
        v_p = 1e-2
        
        %
        threshold = 10;
    end

    properties(DiscreteState)
        rec_id          % recorded beacon id
        
        x               % position
        
        P               % error covariance matrix
        
        init_pos        % initial position where the anchor was observed
        
        directions      %
        direction_dist  %    
        rank            %
    end
    
    methods
        % Constructor
        function obj = detect_uwb_anchor(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:});
        end
    end

    methods(Access = protected)
        function setupImpl(~)
            % Perform one-time calculations, such as computing constants
        end

        function [p_est, dir, rnk] = stepImpl(obj, dist_meas, pos_UAV, id)
            
            p_est = zeros(length(id) ,3);
            dir = zeros(3, 4, length(id));
            rnk = zeros(length(id), 1);
            
            for ii = 1:length(id)
                % Chack whether the marker id belongs to an already seen
                % marker (and is thus already registered) or not.
                if not(ismember(id(ii), obj.rec_id))
                    % If the id is new add a new entry for the discrete
                    % states equal to the actual measure.

                    obj.rec_id(end+1) = id(ii);

                    obj.x(2:4, end+1) = (pos_UAV).' + [dist_meas(ii); 0; 0];
                    obj.x(1, end) = obj.x(2, end)^2 + obj.x(3, end)^2 + obj.x(4, end)^2;
                    obj.x(:,end) = [50^2+20^2, 50, 20, 0].';

                    % If i do directly obj.P(:,:,end+1) = eye(4); on the
                    % first iteration, I obtain an array of size (:,:,2)
                    % and not (:,:,1). The if resolves this issue.
                    if sum(size(obj.P)) == 0
                        obj.P(:,:,1) = 100*eye(4);
                    else
                        obj.P(:,:,end+1) = 100*eye(4);
                    end
                    
                    obj.init_pos(end+1, :) = pos_UAV;
                    
                    if sum(size(obj.directions)) == 0
                        obj.directions(:,:,1) = zeros(3);
                    else
                        obj.directions(:,:,end+1) = zeros(3);
                    end
                    
                    obj.direction_dist(end+1, :) = zeros(1,3);
                    
                    obj.rank(end+1) = 0;
                    
                    
                    index = length(obj.rec_id);
                else
                    % The id is not new ==> directly kalman filter
                    
                    % Find the index corresponding to the id
                    for jj = 1:length(obj.rec_id)
                        if id(ii) == obj.rec_id(jj)
                            index = jj;
                        end
                    end
                end

                if obj.rank(index) >= 0
                    pos_UAV_used = pos_UAV;
                end
                
%                 if obj.rank(index) >= 1
%                     diff1 = dot( obj.x(2:4, index) - pos_UAV_used.', obj.directions(1,:,index).' ) * obj.directions(1,:,index);
%                     pos_UAV_used = pos_UAV_used + diff1;
%                         
%                     dist_meas(ii) = sqrt(dist_meas(ii)^2 - norm(diff1)^2);
%                 end
%                 
%                 if obj.rank(index) >= 2
%                     diff2 = dot( obj.x(2:4, index) - pos_UAV_used.', obj.directions(2,:,index).' ) * obj.directions(2,:,index);
%                     pos_UAV_used = pos_UAV_used + diff2;
%                         
%                     dist_meas(ii) = sqrt(dist_meas(ii)^2 - norm(diff2)^2);
%                 end
                
                if obj.rank(index) < 3
                    % Measurement
                    z = dist_meas(ii)^2 - norm(pos_UAV_used)^2;
                
                    % Observation matrix and covariance matrices
                    H = 2 * [1/2,  -pos_UAV_used(1),  -pos_UAV_used(2),  -pos_UAV_used(3)];
                    R = obj.v_p;
                    Q = diag(obj.w_p*ones(1,4));
                    
                    % Predict P
                    P_pred = obj.P(:,:,index) + Q;
                    
                    % Kalman equations
                    [x_cor, P_cor] = obj.kalman_eq(P_pred, R, H, obj.x(:, index), z);
                    
                    obj.P(:,:,index) = P_cor;
                    
                    
                    % Update the state and the output
                    obj.x(:, index) = x_cor;
                    p_est(ii, :) = obj.x(2:4, index).';
                    
                    
                    % 
%                     switch obj.rank(index)
%                         case 0
%                             if norm( pos_UAV - obj.init_pos(index, :) ) > obj.threshold
%                                 obj.directions(1, :, index) = (pos_UAV - obj.init_pos(index, :))/norm(pos_UAV - obj.init_pos(index, :));
%                                 obj.direction_dist(index, 1) = norm(pos_UAV - obj.init_pos(index, :));
%                                 obj.rank(index) = 1;
%                             end
%                         case 1
%                             temp = (pos_UAV - obj.init_pos(index, :)) - ( dot(pos_UAV - obj.init_pos(index, :), obj.directions(1, :, index) ) *  obj.directions(1, :, index) );
%                             if norm( temp ) > obj.threshold
%                                 obj.directions(2, :, index) = temp/norm(temp);
%                                 obj.direction_dist(index, 2) = norm(temp);
%                                 obj.rank(index) = 2;
%                             end
%                         case 2
%                             temp = (pos_UAV - obj.init_pos(index, :)) - ( dot(pos_UAV - obj.init_pos(index, :), obj.directions(1, :, index) ) *  obj.directions(1, :, index) );
%                             temp = temp - ( dot(temp, obj.directions(2, :, index) ) *  obj.directions(2, :, index) );
%                             if norm( temp ) > obj.threshold
%                                 obj.directions(3, :, index) = temp/norm(temp);
%                                 obj.direction_dist(index, 3) = norm(temp);
%                                 obj.rank(index) = 3;
%                             end 
%                     end
                    
                    
%                     dir(:,:,ii) = [obj.directions(:,:,index), obj.direction_dist(index, :).'];
%                     
%                     rnk(ii) = obj.rank(index);
                else
                    p_est(ii, :) = obj.x(2:4, index).';
                    
                    dir(:,:,ii) = [obj.directions(:,:,index), obj.direction_dist(index, :).'];
                    
                    rnk(ii) = obj.rank(index);
                end
            end
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
            
            obj.rec_id = [];
        
            obj.x = [];
        
            obj.P = [];
            
            obj.init_pos = [];
            
            obj.directions = [];
            
            obj.direction_dist = [];
            
            obj.rank = [];
        end
    end
    
    methods(Static)
        % Kalman equations.
        function [x_corr, P_corr] = kalman_eq(P, R, H, x, z)
            % Innovation covariance
            S = R + H * P * H.';
            
            % Kalman gain
            K = P * H.' / S;
            
            % Error estimate covariance update
            P_corr = P - K * H * P;
            
            x_corr = x + K * ( z - H*x );
        end
    end
end


