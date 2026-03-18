% main.m - 多地图、多算法路径规划高通量仿真框架 (MATLAB 终极全自动定稿版)
clear; clc; close all;

%% 1. 核心实验参数配置
num_runs = 20;           % 独立运行次数 (为了消除随机性，获取严谨的 Mean 和 Std)
max_iter = 100;          % 最大迭代次数
algorithms = {'Ours', 'HO', 'SBOA', 'PSO', 'GWO', 'ARO', 'INFO'}; % 7个算法，主角已更名为 Ours
num_algs = length(algorithms);
num_maps = 5;            % 5个不同复杂度的地图

% 严格对标原刊的学术高级配色 (第一个亮橙色固定给 Ours)
colors = {  [1.00, 0.50, 0.00], [0.00, 0.45, 0.74], [0.47, 0.67, 0.19], ...
            [0.85, 0.33, 0.10], [0.49, 0.18, 0.56], [0.30, 0.75, 0.93], ...
            [0.80, 0.60, 0.80]};

%% 2. 外层大循环：遍历 5 张地图
for map_idx = 1:num_maps
    fprintf('\n=======================================================\n');
    fprintf('>>> 开始测试 Map %d (复杂度级别: %d/5) <<<\n', map_idx, map_idx);
    fprintf('=======================================================\n');
    
    % 获取当前地图和起终点 (依赖你的 GenerateMap 函数)
    [map_grid, start_pt, goal_pt] = GenerateMap(map_idx);
    map_size_x = size(map_grid, 1);
    map_size_y = size(map_grid, 2);
    
    % 初始化数据存储矩阵
    fitness_results = zeros(num_algs, num_runs); 
    time_results = zeros(num_algs, num_runs);
    best_curves = zeros(num_algs, max_iter);
    best_paths = cell(num_algs, 1);
    
    %% 3. 内层循环：遍历 7 个算法
    for alg_idx = 1:num_algs
        alg_name = algorithms{alg_idx};
        fprintf('  运行算法: %-6s ... ', alg_name);
        
        current_alg_best_fit = inf; 
        
        % 独立运行多次求平均
        for run = 1:num_runs
            tic; 
            % 【安全修复】：将中间变量名从 fit 改为 fit_val，彻底避开 MATLAB 内置 fit 函数冲突！
            switch alg_name
                case 'Ours', [path, fit_val, curve] = Ours_Planner(map_grid, start_pt, goal_pt, max_iter);
                case 'HO',   [path, fit_val, curve] = HO_Planner(map_grid, start_pt, goal_pt, max_iter);
                case 'SBOA', [path, fit_val, curve] = SBOA_Planner(map_grid, start_pt, goal_pt, max_iter);
                case 'PSO',  [path, fit_val, curve] = PSO_Planner(map_grid, start_pt, goal_pt, max_iter);
                case 'GWO',  [path, fit_val, curve] = GWO_Planner(map_grid, start_pt, goal_pt, max_iter);
                case 'ARO',  [path, fit_val, curve] = ARO_Planner(map_grid, start_pt, goal_pt, max_iter);
                case 'INFO', [path, fit_val, curve] = INFO_Planner(map_grid, start_pt, goal_pt, max_iter);
            end
            time_taken = toc; 
            
            % 记录当次运行的数据
            fitness_results(alg_idx, run) = fit_val;
            time_results(alg_idx, run) = time_taken;
            
            % 更新并保存本算法的最优曲线和路径用于画图
            if fit_val < current_alg_best_fit
                current_alg_best_fit = fit_val;
                best_curves(alg_idx, :) = curve;
                best_paths{alg_idx} = path;
            end
        end
        fprintf('完成! (最佳适应度: %.4f)\n', current_alg_best_fit);
    end
    
    %% 4. 打印当前地图的统计表格
    fprintf('\n--- Map %d 统计结果 ---\n', map_idx);
    fprintf('算法\t Best\t\t Mean\t\t Worst\t\t Std\t\t Time(s)\n');
    fprintf('---------------------------------------------------------------\n');
    for alg_idx = 1:num_algs
        fits = fitness_results(alg_idx, :);
        times = time_results(alg_idx, :);
        fprintf('%-6s\t %.4f\t %.4f\t %.4f\t %.4f\t %.4f\n', ...
            algorithms{alg_idx}, min(fits), mean(fits), max(fits), std(fits), mean(times));
    end
    fprintf('---------------------------------------------------------------\n');
    
    %% 5. 自动排版出图并极速保存
    fprintf('  正在渲染并导出 Map %d 的高清图片...\n', map_idx);
    
    % ================== 绘制并保存收敛曲线图 ==================
    fig_curve = figure('Name', sprintf('Convergence Map %d', map_idx), 'Position', [100, 100, 650, 500]); 
    hold on; 
    set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1);
    
    curve_handles = zeros(1, num_algs);
    for alg_idx = 1:num_algs
        % 【作图心机】：Ours 算法自动加粗突出显示
        if strcmp(algorithms{alg_idx}, 'Ours')
            curve_handles(alg_idx) = plot(1:max_iter, best_curves(alg_idx, :), 'Color', colors{alg_idx}, 'LineWidth', 3.0); 
        else
            curve_handles(alg_idx) = plot(1:max_iter, best_curves(alg_idx, :), 'Color', colors{alg_idx}, 'LineWidth', 1.5); 
        end
    end
    xlim([0, max_iter]);
    set(gca, 'YScale', 'log'); % 开启对数坐标
    
    xlabel('(a). Convergence diagram.', 'FontSize', 12, 'FontWeight', 'normal', 'Color', 'k');
    ylabel('Fitness Value (Log Scale)', 'FontSize', 11);
    
    leg_curve = legend(curve_handles, algorithms, 'Location', 'northeast');
    set(leg_curve, 'FontSize', 10, 'Box', 'on', 'EdgeColor', 'k'); 
    
    % 杀手锏 exportgraphics 自动保存
    exportgraphics(fig_curve, sprintf('Convergence_Map%d.png', map_idx), 'Resolution', 300);
    close(fig_curve); % 保存完毕自动关闭窗口，释放内存
    
    % ================== 绘制并保存栅格路径图 ==================
    fig_path = figure('Name', sprintf('Path Map %d', map_idx), 'Position', [200, 150, 600, 600]); 
    hold on; axis equal;
    colormap([1 1 1; 0.76 0.60 0.82]); 
    imagesc(map_grid'); 
    set(gca, 'YDir', 'normal'); 
    
    rectangle('Position', [0.5, 0.5, map_size_x, map_size_y], 'EdgeColor', 'k', 'LineWidth', 1);
    
    path_handles = zeros(1, num_algs);
    for alg_idx = 1:num_algs
        path = best_paths{alg_idx};
        % 【作图心机】：Ours 算法自动加粗突出显示
        if strcmp(algorithms{alg_idx}, 'Ours')
            path_handles(alg_idx) = plot(path(:, 1), path(:, 2), 'Color', colors{alg_idx}, 'LineWidth', 2.5); 
        else
            path_handles(alg_idx) = plot(path(:, 1), path(:, 2), 'Color', colors{alg_idx}, 'LineWidth', 1.5); 
        end
    end
    
    plot(start_pt(1), start_pt(2), 'ro', 'MarkerSize', 9, 'MarkerFaceColor', 'r', 'HandleVisibility', 'off');
    plot(goal_pt(1), goal_pt(2), 'rs', 'MarkerSize', 9, 'MarkerFaceColor', 'r', 'HandleVisibility', 'off');
    
    xlabel('(b). Road map.', 'FontSize', 12, 'FontWeight', 'normal', 'Color', 'k');
    
    leg_path = legend(path_handles, algorithms, 'Location', 'northwest');
    set(leg_path, 'FontSize', 10, 'Box', 'on', 'EdgeColor', 'k');
    
    axis([0.5, map_size_x+0.5, 0.5, map_size_y+0.5]);
    set(gca, 'XTick', 0:10:map_size_x, 'YTick', 0:10:map_size_y); 
    
    % 杀手锏 exportgraphics 自动保存
    exportgraphics(fig_path, sprintf('Path_Map%d.png', map_idx), 'Resolution', 300);
    close(fig_path); 
end

fprintf('\n>>> 🎉 所有地图仿真实验圆满结束！10张绝美期刊级大图已安静地躺在您的文件夹中！ <<<\n');