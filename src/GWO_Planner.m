function [best_path, best_fitness, convergence_curve] = GWO_Planner(map_grid, start_pt, goal_pt, max_iter)
    % GWO (Grey Wolf Optimizer) 路径规划
    % 【已升级：统一标准适应度函数，包含五次方动态惩罚与静态基线】
    
    pop_size = 50;           
    num_waypoints = 5;       
    dim = num_waypoints * 2; 
    
    x_max = size(map_grid, 1); y_max = size(map_grid, 2);
    lb = ones(1, dim);       
    ub = repmat([x_max, y_max], 1, num_waypoints); 
    X = repmat(lb, pop_size, 1) + rand(pop_size, dim) .* repmat((ub - lb), pop_size, 1);
    
    % 初始化 Alpha, Beta, Delta 狼 (基于动态适应度，用于引领狼群)
    Alpha_pos = zeros(1, dim); Alpha_score_dyn = inf;
    Beta_pos  = zeros(1, dim); Beta_score_dyn  = inf;
    Delta_pos = zeros(1, dim); Delta_score_dyn = inf;
    
    % 记录用于展示和对比的绝对全局最优 (基于静态适应度)
    best_static_score = inf;
    best_static_pos = zeros(1, dim);
    
    convergence_curve = zeros(1, max_iter);
    
    for t = 1:max_iter
        % 1. 评估所有灰狼的适应度
        for i = 1:pop_size
            % 传入 t 和 max_iter，获取动态和静态适应度
            [current_fit_dyn, current_fit_static] = EvaluatePath(X(i, :), start_pt, goal_pt, map_grid, t, max_iter);
            
            % 更新用于科研图表的全局绝对最优 (静态分数)
            if current_fit_static < best_static_score
                best_static_score = current_fit_static;
                best_static_pos = X(i, :);
            end
            
            % 更新三大头狼的位置 (必须使用动态分数，以实现后期的核爆驱赶效果)
            if current_fit_dyn < Alpha_score_dyn
                Delta_score_dyn = Beta_score_dyn; Delta_pos = Beta_pos;
                Beta_score_dyn = Alpha_score_dyn; Beta_pos = Alpha_pos;
                Alpha_score_dyn = current_fit_dyn; Alpha_pos = X(i, :);
            elseif current_fit_dyn > Alpha_score_dyn && current_fit_dyn < Beta_score_dyn
                Delta_score_dyn = Beta_score_dyn; Delta_pos = Beta_pos;
                Beta_score_dyn = current_fit_dyn; Beta_pos = X(i, :);
            elseif current_fit_dyn > Alpha_score_dyn && current_fit_dyn > Beta_score_dyn && current_fit_dyn < Delta_score_dyn
                Delta_score_dyn = current_fit_dyn; Delta_pos = X(i, :);
            end
        end
        
        a = 2 - t * (2 / max_iter); % a 从 2 线性递减到 0
        
        % 2. 头狼带领狼群围剿猎物 (位置更新)
        for i = 1:pop_size
            for j = 1:dim
                r1 = rand(); r2 = rand();
                A1 = 2 * a * r1 - a; C1 = 2 * r2;
                D_alpha = abs(C1 * Alpha_pos(j) - X(i, j));
                X1 = Alpha_pos(j) - A1 * D_alpha;
                
                r1 = rand(); r2 = rand();
                A2 = 2 * a * r1 - a; C2 = 2 * r2;
                D_beta = abs(C2 * Beta_pos(j) - X(i, j));
                X2 = Beta_pos(j) - A2 * D_beta;
                
                r1 = rand(); r2 = rand();
                A3 = 2 * a * r1 - a; C3 = 2 * r2;
                D_delta = abs(C3 * Delta_pos(j) - X(i, j));
                X3 = Delta_pos(j) - A3 * D_delta;
                
                X(i, j) = (X1 + X2 + X3) / 3;
            end
            % 越界限制
            X(i, :) = max(X(i, :), lb);
            X(i, :) = min(X(i, :), ub);
        end
        
        % 记录收敛曲线 (使用静态最优分数)
        convergence_curve(t) = best_static_score;
    end
    
    best_fitness = best_static_score;
    best_path = [start_pt; reshape(best_static_pos, 2, num_waypoints)'; goal_pt];
end

% =========================================================================
% 统一标准局部辅助函数: 适应度计算 (与 Ours_Planner 100% 相同)
% =========================================================================
function [fit_dyn, fit_static] = EvaluatePath(agent_pos, start_pt, goal_pt, map_grid, t, max_iter)
    num_waypoints = length(agent_pos) / 2;
    waypoints = reshape(agent_pos, 2, num_waypoints)';
    full_path = [start_pt; waypoints; goal_pt];
    
    path_length = 0; 
    collision_count = 0; 
    smoothness_cost = 0; 
    
    % 1. 计算距离与碰撞
    for k = 1:(size(full_path, 1) - 1)
        pt1 = full_path(k, :); 
        pt2 = full_path(k+1, :);
        dist = norm(pt1 - pt2);
        path_length = path_length + dist;
        
        num_samples = ceil(dist * 3); 
        if num_samples > 0
            for s = 0:num_samples
                sample_pt = pt1 + (pt2 - pt1) * (s / num_samples);
                grid_x = round(sample_pt(1)); 
                grid_y = round(sample_pt(2));
                if grid_x >= 1 && grid_x <= size(map_grid,1) && grid_y >= 1 && grid_y <= size(map_grid,2)
                    if map_grid(grid_x, grid_y) == 1
                        collision_count = collision_count + 1; 
                    end
                else
                    collision_count = collision_count + 1; 
                end
            end
        end
    end
    
    % 2. 计算平滑度 (统一所有算法的评价标准)
    for k = 2:(size(full_path, 1) - 1)
        v1 = full_path(k, :) - full_path(k-1, :);
        v2 = full_path(k+1, :) - full_path(k, :);
        norm_v1 = norm(v1); norm_v2 = norm(v2);
        if norm_v1 > 1e-5 && norm_v2 > 1e-5
            cos_theta = dot(v1, v2) / (norm_v1 * norm_v2);
            cos_theta = max(min(cos_theta, 1), -1); 
            smoothness_cost = smoothness_cost + (1 - cos_theta);
        end
    end
    
    smooth_weight = 2.0; 
    
    % =================================================================
    % 1. 自适应动态惩罚 (五次方核爆指数) - 用于物竞天择淘汰
    % =================================================================
    dyn_penalty_weight = 500 + 99500 * (t / max_iter)^5;
    fit_dyn = path_length + smooth_weight * smoothness_cost + dyn_penalty_weight * collision_count;
    
    % =================================================================
    % 2. 视觉级静态惩罚 (1000) - 用于记录科学对比的最终成绩
    % =================================================================
    static_penalty_weight = 1000; 
    fit_static = path_length + static_penalty_weight * collision_count;
end