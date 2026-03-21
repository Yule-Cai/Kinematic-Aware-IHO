function [best_path, best_fitness, convergence_curve] = Ours_Planner(map_grid, start_pt, goal_pt, max_iter)
    % Ours_Planner: 动力学感知 IHO (Kinematic-Aware IHO)
    % 【V5 绝对领域版】：限制重启窗口 + 10向局部爆破搜索，强行抹除一切微小擦碰，追求全满分！
    
    pop_size = 50;           
    num_waypoints = 5;       
    dim = num_waypoints * 2; 
    
    x_max = size(map_grid, 1); 
    y_max = size(map_grid, 2);
    lb = ones(1, dim);       
    ub = repmat([x_max, y_max], 1, num_waypoints); 
    map_scale = max(x_max, y_max);
    
    %% 1. 初始化
    pool_size = pop_size * 100; 
    X_pool = zeros(pool_size, dim);
    for i = 1:pool_size
        noise_factor = rand(); 
        for w = 1:num_waypoints
            alpha = w / (num_waypoints + 1);
            base_pt = start_pt + alpha * (goal_pt - start_pt);
            noise = (rand(1, 2) - 0.5) * (map_scale * noise_factor); 
            X_pool(i, (w*2-1):w*2) = base_pt + noise;
        end
    end
    X_pool = max(X_pool, repmat(lb, pool_size, 1));
    X_pool = min(X_pool, repmat(ub, pool_size, 1));
    
    fit_dyn_pool = zeros(pool_size, 1);
    fit_static_pool = zeros(pool_size, 1);
    
    for i = 1:pool_size
        [fit_dyn_pool(i), fit_static_pool(i)] = EvaluatePath(X_pool(i, :), start_pt, goal_pt, map_grid, 1, max_iter);
    end
    
    [~, sort_idx] = sort(fit_dyn_pool); 
    X = X_pool(sort_idx(1:pop_size), :);
    fit_X_dyn = fit_dyn_pool(sort_idx(1:pop_size));
    
    best_fitness = min(fit_static_pool);
    best_idx = find(fit_static_pool == best_fitness, 1);
    best_position = X_pool(best_idx, :);
    
    convergence_curve = zeros(1, max_iter);
    
    stall_counter = 0; 
    last_best_fitness = best_fitness;
    Stagnation_Limit = 5; 
    
    %% 2. 核心迭代
    for t = 1:max_iter
        for i = 1:pop_size
            [fit_X_dyn(i), current_static] = EvaluatePath(X(i, :), start_pt, goal_pt, map_grid, t, max_iter);
            if current_static < best_fitness
                best_fitness = current_static;
                best_position = X(i, :);
            end
        end
        
        if best_fitness >= 1000 && abs(best_fitness - last_best_fitness) < 1e-3
            stall_counter = stall_counter + 1;
        else
            stall_counter = 0; 
        end
        last_best_fitness = best_fitness;
        
        [~, current_sort_idx] = sort(fit_X_dyn);
        X = X(current_sort_idx, :);
        fit_X_dyn = fit_X_dyn(current_sort_idx);
        
        best1 = X(1, :); best2 = X(min(2, pop_size), :); best3 = X(min(3, pop_size), :);
        a = 2 * (1 - t / max_iter); 
        elite_ratio = 0.3 * exp(-stall_counter / 3); 
        num_elites = max(2, round(pop_size * elite_ratio)); 
        
        % 【护城河 1】：限制大清洗只在 20% ~ 70% 的窗口进行，最后 30% 绝对不捣乱！
        if stall_counter >= Stagnation_Limit && t > 0.2 * max_iter && t < 0.7 * max_iter
            for j = round(pop_size * 0.3):pop_size
                for w = 1:num_waypoints
                    alpha = w / (num_waypoints + 1);
                    base_pt = start_pt + alpha * (goal_pt - start_pt);
                    X(j, (w*2-1):w*2) = base_pt + (rand(1, 2) - 0.5) * map_scale * 0.9; 
                end
                X(j, :) = max(X(j, :), lb); X(j, :) = min(X(j, :), ub);
                [fit_X_dyn(j), ~] = EvaluatePath(X(j, :), start_pt, goal_pt, map_grid, t, max_iter);
            end
            [~, current_sort_idx] = sort(fit_X_dyn);
            X = X(current_sort_idx, :);
            fit_X_dyn = fit_X_dyn(current_sort_idx);
            best1 = X(1, :); 
            stall_counter = 0; 
        end
        
        for i = 1:pop_size
            if i <= num_elites
                r_guide = rand();
                if r_guide < 0.4; guide_pt = best1; elseif r_guide < 0.8; guide_pt = best2; else; guide_pt = best3; end
                r1 = rand(1, dim); r2 = rand(1, dim);
                A = 2 * a .* r1 - a; C = 2 .* r2;
                D = abs(C .* guide_pt - X(i, :));
                X_new_i = guide_pt - A .* D; 
            else
                if stall_counter > 2 && t < 0.8 * max_iter
                    X_new_i = X(i, :) + map_scale * 0.3 * randn(1, dim); 
                else
                    p_explore = 0.8 * (1 - t / max_iter); 
                    if rand() < p_explore
                        rand1 = randi(pop_size); while rand1 == i; rand1 = randi(pop_size); end
                        rand2 = randi(pop_size); while rand2 == i || rand2 == rand1; rand2 = randi(pop_size); end
                        X_new_i = X(i, :) + rand(1, dim) .* (best1 - X(i, :)) + (a / 2) * (X(rand1, :) - X(rand2, :));
                    else
                        X_new_i = best1 + randn(1, dim) .* (ub - lb) * 0.005 * a;
                    end
                end
            end
            X_new_i = max(X_new_i, lb); X_new_i = min(X_new_i, ub);
            [fit_new_dyn, fit_new_static] = EvaluatePath(X_new_i, start_pt, goal_pt, map_grid, t, max_iter);
            if fit_new_dyn < fit_X_dyn(i)
                X(i, :) = X_new_i; fit_X_dyn(i) = fit_new_dyn;
                if fit_new_static < best_fitness
                    best_fitness = fit_new_static; best_position = X_new_i;
                end
            end
        end
        
        % =================================================================
        % 【护城河 2】：十向局部暴搜防擦碰 (10-Direction Radial Search)
        % =================================================================
        if t > 0.65 * max_iter 
            temp_best = best_position;
            best_waypoints = reshape(temp_best, 2, num_waypoints)';
            Probe_Step = map_scale * 0.05; 
            
            for ironing_round = 1:5 
                for w = 1:num_waypoints
                    if w == 1; pt_prev = start_pt; else; pt_prev = best_waypoints(w-1, :); end
                    if w == num_waypoints; pt_next = goal_pt; else; pt_next = best_waypoints(w+1, :); end
                    
                    mid_pt = (pt_prev + pt_next) / 2.0;
                    candidate = temp_best;
                    
                    if best_fitness >= 1000
                        % 还在撞墙！直接在周围随机撒 10 个点进行暴力探测
                        best_probe_dyn = inf;
                        best_probe_static = inf;
                        best_probe_pt = mid_pt;
                        
                        for attempt = 1:10
                            test_cand = candidate;
                            % 产生全方位的微小随机偏移
                            test_cand((w*2-1):w*2) = mid_pt + randn(1,2) * Probe_Step;
                            test_cand = max(test_cand, lb); test_cand = min(test_cand, ub);
                            [p_dyn, p_static] = EvaluatePath(test_cand, start_pt, goal_pt, map_grid, t, max_iter);
                            
                            if p_dyn < best_probe_dyn
                                best_probe_dyn = p_dyn;
                                best_probe_static = p_static;
                                best_probe_pt = test_cand((w*2-1):w*2);
                            end
                        end
                        candidate((w*2-1):w*2) = best_probe_pt;
                        fit_cand_dyn = best_probe_dyn;
                        fit_cand_static = best_probe_static;
                    else
                        % 绝对安全了，放心拉直
                        candidate((w*2-1):w*2) = mid_pt; 
                        candidate = max(candidate, lb); candidate = min(candidate, ub);
                        [fit_cand_dyn, fit_cand_static] = EvaluatePath(candidate, start_pt, goal_pt, map_grid, t, max_iter);
                    end
                    
                    if fit_cand_static < best_fitness
                        temp_best = candidate;
                        best_fitness = fit_cand_static;
                        best_position = temp_best;
                        X(1, :) = temp_best;
                        fit_X_dyn(1) = fit_cand_dyn;
                        best_waypoints = reshape(temp_best, 2, num_waypoints)'; 
                    end
                end
            end
        end
        convergence_curve(t) = best_fitness;
    end
    best_path = [start_pt; reshape(best_position, 2, num_waypoints)'; goal_pt];
end

function [fit_dyn, fit_static] = EvaluatePath(agent_pos, start_pt, goal_pt, map_grid, t, max_iter)
    num_waypoints = length(agent_pos) / 2;
    waypoints = reshape(agent_pos, 2, num_waypoints)';
    full_path = [start_pt; waypoints; goal_pt];
    
    path_length = 0; collision_count = 0; smoothness_cost = 0; 
    
    for k = 1:(size(full_path, 1) - 1)
        pt1 = full_path(k, :); pt2 = full_path(k+1, :);
        dist = norm(pt1 - pt2); path_length = path_length + dist;
        num_samples = ceil(dist * 3); 
        if num_samples > 0
            for s = 0:num_samples
                sample_pt = pt1 + (pt2 - pt1) * (s / num_samples);
                grid_x = round(sample_pt(1)); grid_y = round(sample_pt(2));
                if grid_x >= 1 && grid_x <= size(map_grid,1) && grid_y >= 1 && grid_y <= size(map_grid,2)
                    if map_grid(grid_x, grid_y) == 1, collision_count = collision_count + 1; end
                else
                    collision_count = collision_count + 1; 
                end
            end
        end
    end
    
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
    dyn_penalty_weight = 500 + 99500 * (t / max_iter)^5;
    fit_dyn = path_length + smooth_weight * smoothness_cost + dyn_penalty_weight * collision_count;
    static_penalty_weight = 1000; 
    fit_static = path_length + static_penalty_weight * collision_count;
end