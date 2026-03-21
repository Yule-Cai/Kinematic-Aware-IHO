% run_statistical_experiments.m - 终极防御版 (含20次原始数据完整导出与自动日志)
clear; clc; close all;

%% 0. 自动化日志记录 (专为 GitHub 开源和自查设计)
if ~exist('logs', 'dir'), mkdir('logs'); end
time_str = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
log_filename = fullfile('logs', sprintf('Experiment_Log_%s.txt', time_str));
diary(log_filename); diary on;

%% 1. 核心实验参数配置
num_runs = 20;           % 独立运行次数
max_iter = 200;          % 最大迭代次数
algorithms = {'Ours', 'HO', 'SBOA', 'PSO', 'GWO', 'ARO', 'INFO'}; 
num_algs = length(algorithms);
num_maps = 5;            % 5张地图

% 动态生成 Excel 表头 (7 列统计指标 + 20 列原始数据)
headers = {'Map', 'Algorithm', 'Best', 'Worst', 'Mean', 'Std', 'Avg_Time(s)'};
for i = 1:num_runs
    headers{end+1} = sprintf('Run_%d', i); % 自动生成 Run_1, Run_2... 列名
end

% 初始化 Excel 数据存储矩阵 (35行数据 + 1行表头)
results_data = cell(num_maps * num_algs + 1, length(headers));
results_data(1, :) = headers;
excel_row = 2; % 从第二行开始写入

fprintf('=======================================================\n');
fprintf('🚀 动力学感知 IHO 算法 - 全通量原始数据采集与统计实验\n');
fprintf('📅 实验时间: %s\n', char(datetime('now')));
fprintf('⚙️  参数设置: %d 张地图 | %d 个算法 | 每组独立运行 %d 次\n', num_maps, num_algs, num_runs);
fprintf('=======================================================\n');

%% 2. 开始大循环
for map_idx = 1:num_maps
    fprintf('\n>>> 开始测试 Map %d <<<\n', map_idx);
    
    % 获取当前地图和起终点
    [map_grid, start_pt, goal_pt] = GenerateMap(map_idx);
    
    for alg_idx = 1:num_algs
        alg_name = algorithms{alg_idx};
        fprintf('  [Map %d] 正在运行: %-6s | 进度: ', map_idx, alg_name);
        
        % 初始化记录数组
        fitness_history = zeros(1, num_runs);
        time_history = zeros(1, num_runs);
        
        for run = 1:num_runs
            fprintf('%d.', run); 
            
            tic; 
            switch alg_name
                case 'Ours', [~, fit_val, ~] = Ours_Planner(map_grid, start_pt, goal_pt, max_iter);
                case 'HO',   [~, fit_val, ~] = HO_Planner(map_grid, start_pt, goal_pt, max_iter);
                case 'SBOA', [~, fit_val, ~] = SBOA_Planner(map_grid, start_pt, goal_pt, max_iter);
                case 'PSO',  [~, fit_val, ~] = PSO_Planner(map_grid, start_pt, goal_pt, max_iter);
                case 'GWO',  [~, fit_val, ~] = GWO_Planner(map_grid, start_pt, goal_pt, max_iter);
                case 'ARO',  [~, fit_val, ~] = ARO_Planner(map_grid, start_pt, goal_pt, max_iter);
                case 'INFO', [~, fit_val, ~] = INFO_Planner(map_grid, start_pt, goal_pt, max_iter);
            end
            time_history(run) = toc; 
            
            % 记录每一次的原始 Fitness 值！
            fitness_history(run) = fit_val; 
        end
        
        % 3. 计算统计指标
        b_val = min(fitness_history);
        w_val = max(fitness_history);
        m_val = mean(fitness_history);
        s_val = std(fitness_history);
        t_val = mean(time_history);
        
        fprintf(' | 完成! Mean: %.4f, Std: %.4f\n', m_val, s_val);
        
        % 4. 写入 Excel 数据矩阵 
        % 前 7 列存统计结果
        results_data(excel_row, 1:7) = {map_idx, alg_name, b_val, w_val, m_val, s_val, t_val};
        % 后 20 列存每一次的原始数据
        results_data(excel_row, 8:end) = num2cell(fitness_history); 
        
        excel_row = excel_row + 1;
    end
end

%% 5. 导出 Excel 并结束日志
excel_filename = 'Statistical_Results_Raw_Data.xlsx';
writecell(results_data, excel_filename);

fprintf('\n=======================================================\n');
fprintf('✅ 所有实验圆满完成！\n');
fprintf('📊 包含全部 700 个原始数据的表格已导出至: %s\n', excel_filename);
fprintf('📝 实验日志已保存至: %s\n', log_filename);
fprintf('=======================================================\n');

diary off;