% run_ablation.m  ——  消融实验全自动驱动脚本
%
% 功能：在 5 张地图 × 5 个消融变体上各独立运行 num_runs 次，
%       统计 SR / Valid Mean / Valid Worst / Valid Std，
%       并自动导出：
%         1. Ablation_Table.xlsx  (完整数值统计表)
%         2. Ablation_Convergence_MapX.png  (各地图收敛曲线对比)
%
% 使用前提：
%   将本文件与 Ablation_Planner.m 以及原项目的
%   GenerateMap.m、map.m 放在同一目录，然后在 MATLAB 中运行。
%
% 作者: Yule Cai  |  消融版整理: 2026-04

clear; clc; close all;

%% ── 0. 实验配置 ──────────────────────────────────────────────────────────────
num_runs  = 20;
max_iter  = 100;
num_maps  = 5;
COLLISION_THRESHOLD = 1000;   % fit_static >= 此值视为失败（有碰撞）

% ── 变体定义 ──
%   每列：[use_kinematic, use_ironing, use_adaptive_penalty]
variants = {
    'Var-A (Baseline)',   false, false, false;
    'Var-B (+Kine)',       true,  false, false;
    'Var-C (+Kine+Lap)',   true,  true,  false;
    'Var-D (+Kine+Adp)',   true,  false, true;
    'Full IHO',            true,  true,  true;
};
num_variants = size(variants, 1);

% 绘图配色（与主论文风格一致）
colors = {
    [0.50, 0.50, 0.50], ...   % Var-A  灰
    [0.00, 0.45, 0.74], ...   % Var-B  蓝
    [0.47, 0.67, 0.19], ...   % Var-C  绿
    [0.85, 0.33, 0.10], ...   % Var-D  红
    [1.00, 0.50, 0.00],  ...  % Full   橙（主角色）
};

%% ── 1. 结果存储矩阵 ──────────────────────────────────────────────────────────
% fit_raw(v, m, r)  ：第 v 变体、第 m 地图、第 r 次运行的 fit_static 值
fit_raw       = zeros(num_variants, num_maps, num_runs);
best_curves   = zeros(num_variants, num_maps, max_iter);

%% ── 2. 主循环：变体 × 地图 × 运行 ───────────────────────────────────────────
for m = 1:num_maps
    fprintf('\n══════════════════════════════════════════\n');
    fprintf('  Map %d / %d\n', m, num_maps);
    fprintf('══════════════════════════════════════════\n');

    [map_grid, start_pt, goal_pt] = GenerateMap(m);

    for v = 1:num_variants
        vname     = variants{v, 1};
        use_k     = variants{v, 2};
        use_i     = variants{v, 3};
        use_a     = variants{v, 4};

        fprintf('  %-22s ... ', vname);

        this_best_fit   = inf;
        this_best_curve = zeros(1, max_iter);

        for r = 1:num_runs
            [~, fit_val, curve] = Ablation_Planner( ...
                map_grid, start_pt, goal_pt, max_iter, ...
                use_k, use_i, use_a);

            fit_raw(v, m, r) = fit_val;

            if fit_val < this_best_fit
                this_best_fit   = fit_val;
                this_best_curve = curve;
            end
        end

        best_curves(v, m, :) = this_best_curve;
        fprintf('完成 (最佳 fit = %.4f)\n', this_best_fit);
    end
end

%% ── 3. 统计计算 ─────────────────────────────────────────────────────────────
% 输出表格列：Variant | Map | SR(%) | Valid_Mean | Valid_Worst | Valid_Std
result_rows = {};   % 用于写入 Excel

for v = 1:num_variants
    for m = 1:num_maps
        runs_data = squeeze(fit_raw(v, m, :));  % num_runs × 1

        success_mask = runs_data < COLLISION_THRESHOLD;
        SR           = sum(success_mask) / num_runs * 100;

        valid_data   = runs_data(success_mask);
        if isempty(valid_data)
            valid_mean  = NaN;
            valid_worst = NaN;
            valid_std   = NaN;
        else
            valid_mean  = mean(valid_data);
            valid_worst = max(valid_data);
            valid_std   = std(valid_data);
        end

        result_rows(end+1, :) = { ...
            variants{v,1}, ...
            sprintf('Map %d', m), ...
            SR, valid_mean, valid_worst, valid_std };
    end
end

%% ── 4. 打印控制台摘要 ───────────────────────────────────────────────────────
fprintf('\n\n══════════════ 消融实验统计结果 ══════════════\n');
fprintf('%-24s %-8s %6s  %10s  %10s  %8s\n', ...
    'Variant', 'Map', 'SR(%)', 'Valid_Mean', 'Valid_Worst', 'Valid_Std');
fprintf('%s\n', repmat('-', 1, 72));
for row = 1:size(result_rows, 1)
    fprintf('%-24s %-8s %6.1f  %10.4f  %10.4f  %8.4f\n', ...
        result_rows{row, 1}, result_rows{row, 2}, ...
        result_rows{row, 3}, result_rows{row, 4}, ...
        result_rows{row, 5}, result_rows{row, 6});
    % 每隔 num_maps 行打一条分隔线
    if mod(row, num_maps) == 0
        fprintf('%s\n', repmat('-', 1, 72));
    end
end

%% ── 5. 导出 Excel ────────────────────────────────────────────────────────────
headers = {'Variant', 'Map', 'SR(%)', 'Valid_Mean', 'Valid_Worst', 'Valid_Std'};
out_table = cell2table(result_rows, 'VariableNames', headers);
writetable(out_table, 'Ablation_Table.xlsx', 'Sheet', 'Ablation');
fprintf('\n✅ 统计表已导出 → Ablation_Table.xlsx\n');

%% ── 6. 绘制并导出收敛曲线（每张地图一张图）────────────────────────────────
for m = 1:num_maps
    fig = figure('Name', sprintf('Ablation Convergence Map %d', m), ...
                 'Position', [100, 100, 700, 480]);
    hold on;
    set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1);

    h = zeros(1, num_variants);
    for v = 1:num_variants
        curve_data = squeeze(best_curves(v, m, :))';
        lw = 1.5;
        ls = '-';
        if v == num_variants          % Full IHO 加粗
            lw = 3.0;
        elseif v == 1                 % Baseline 用虚线
            ls = '--';
        end
        h(v) = plot(1:max_iter, curve_data, ...
            'Color', colors{v}, 'LineWidth', lw, 'LineStyle', ls);
    end

    set(gca, 'YScale', 'log');
    xlim([1, max_iter]);
    xlabel('Iteration', 'FontSize', 12);
    ylabel('Fitness Value (Log Scale)', 'FontSize', 12);
    title(sprintf('Ablation Study — Map %d Convergence', m), 'FontSize', 13);

    leg_labels = variants(:, 1);
    legend(h, leg_labels, 'Location', 'northeast', 'FontSize', 10, ...
           'Box', 'on', 'EdgeColor', 'k');

    fname = sprintf('Ablation_Convergence_Map%d.png', m);
    exportgraphics(fig, fname, 'Resolution', 300);
    close(fig);
    fprintf('✅ 收敛曲线已导出 → %s\n', fname);
end

fprintf('\n🎉 消融实验全部完成！\n');
fprintf('   请检查当前目录中的 Ablation_Table.xlsx 和 Ablation_Convergence_MapX.png\n');