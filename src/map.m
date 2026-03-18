% =========================================================================
% 一键导出 5 张 SCI 纯净版地图 (用于流程图 Phase 1)
% 依赖函数: GenerateMap.m (你的原始生成代码)
% =========================================================================
clc; clear; close all;

% 循环生成 Level 1 到 Level 5 的地图
for level = 1:5
    % 1. 调用你的原始代码生成地图数据
    [map_grid, start_pt, goal_pt] = GenerateMap(level);
    
    % 2. 创建一个隐藏的图窗 (不弹窗打扰，直接后台生成)
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100, 100, 600, 600]);
    hold on;
    
    % 3. 绘制地图矩阵
    % 设置颜色：0 为纯白(自由空间)，1 为柔和的紫色(障碍物)
    customColormap = [1 1 1; 0.5 0.4 0.7]; 
    
    % 注意：MATLAB 矩阵的行列对应关系，通常需要转置以符合常规的 XY 坐标系
    imagesc(map_grid'); 
    colormap(customColormap);
    axis xy; % 将坐标原点设置在左下角
    
    % 4. 绘制起点和终点 (与你的流程图图例严格对应)
    % 起点：红色实心圆圈 (Red Circle)
    plot(start_pt(1), start_pt(2), 'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k', 'LineWidth', 1);
    
    % 终点：红色实心方块 (Red Square)
    plot(goal_pt(1), goal_pt(2), 'rs', 'MarkerSize', 12, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k', 'LineWidth', 1);
    
    % 5. 去除坐标轴、刻度和白边
    axis equal;
    axis off;
    
    % 严丝合缝地框住地图，不留多余空白
    map_size = length(map_grid);
    xlim([0.5, map_size + 0.5]);
    ylim([0.5, map_size + 0.5]);
    
    % 6. 导出为 300 dpi 高清图片
    outputFileName = sprintf('Phase1_Map_Level_%d.png', level);
    exportgraphics(fig, outputFileName, 'Resolution', 300);
    
    close(fig);
    fprintf('成功导出: %s\n', outputFileName);
end

disp('✅ 5 张纯净版地图已全部导出完毕！可以直接贴入 PPT 流程图！');