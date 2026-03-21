function [best_path, best_fitness, convergence_curve] = INFO_Planner(map_grid, start_pt, goal_pt, max_iter)
    % INFO (Weighted Mean of Vectors, 2022) 路径规划
    % 【已升级：统一标准适应度函数，包含五次方动态惩罚与静态基线】
    
    pop_size = 50;           
    num_waypoints = 5;       
    dim = num_waypoints * 2; 
    
    x_max = size(map_grid, 1); y_max = size(map_grid, 2);
    lb = ones(1, dim);       
    ub = repmat([x_max, y_max], 1, num_waypoints); 
    X = repmat(lb, pop_size, 1) + rand(pop_size, dim) .* repmat((ub - lb), pop_size, 1);
    
    % 记录用于展示和对比的绝对全局最优 (静态分数)
    best_static_score = inf;
    best_static_pos = zeros(1, dim);
    
    % 记录用于内部排序淘汰的动态适应度
    fit_dyn_array = inf(pop_size, 1);
    
    convergence_curve = zeros(1, max_iter);
    
    for t = 1:max_iter
        % 1. 评估当前种群的动态和静态适应度
        for i = 1:pop_size
            [fit_dyn_array(i), current_fit_static] = EvaluatePath(X(i, :), start_pt, goal_pt, map_grid, t, max_iter);
            
            % 更新用于科研图表的全局绝对最优 (静态分数)
            if current_fit_static < best_static_score
                best_static_score = current_fit_static;
                best_static_pos = X(i, :);
            end
        end
        
        % 2. 计算 INFO 的基本向量 (必须基于动态分数排序，实现后期逼退越界个体的效果)
        mean_rule = mean(X); 
        [~, sorted_idx] = sort(fit_dyn_array);
        best_X = X(sorted_idx(1), :);
        better_X = X(sorted_idx(2), :);
        worst_X = X(sorted_idx(end), :); % 后期撞墙的个体必定沦为 worst_X
        
        alpha = 2 * exp(-4 * (t / max_iter)); % 缩放因子
        
        % 3. 位置更新策略
        for i = 1:pop_size
            r1 = rand(); r2 = rand();
            if r1 < 0.5
                % 基于较好解的更新
                WM = (r2 * best_X + (1 - r2) * better_X) / 2;
                X_new = WM + alpha * randn(1, dim) .* (best_X - X(i, :));
            else
                % 基于平均规则的更新
                X_new = mean_rule + alpha * randn(1, dim) .* (X(i, :) - worst_X);
            end
            
            % 越界限制
            X_new = max(X_new, lb);
            X_new = min(X_new, ub);
            
            % 严密贪婪选择：评估新位置
            [new_fit_dyn, new_fit_static] = EvaluatePath(X_new, start_pt, goal_pt, map_grid, t, max_iter);
            
            % 用动态适应度进行内部优胜劣汰
            if new_fit_dyn < fit_dyn_array(i)
                X(i, :) = X_new;
                fit_dyn_array(i) = new_fit_dyn; % 更新数组
                
                % 顺便检查静态记录
                if new_fit_static < best_static_score
                    best_static_score = new_fit_static;
                    best_static_pos = X_new;
                end
            end
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
    
    % 2. 计算平滑度
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