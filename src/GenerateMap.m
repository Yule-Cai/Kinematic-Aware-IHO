function [map_grid, start_pt, goal_pt] = GenerateMap(level)
    % 动态生成与参考论文完全一致的随机散落栅格障碍物地图

    % 根据难度等级设置地图尺寸和障碍物区块数量
    if level <= 3
        map_size = 40;  % 前3张图对标 40x40 真实环境
        num_blocks = 30 + level * 25; % 障碍物块数: L1=55, L2=80, L3=105
    else
        map_size = 80;  % 后2张图对标 80x80 极端复杂环境
        num_blocks = 150 + (level-4) * 100; % 障碍物块数: L4=150, L5=250
    end

    map_grid = zeros(map_size, map_size);
    start_pt = [2, 2];
    goal_pt = [map_size-2, map_size-2];

    % 兼容 Octave 的固定随机种子，确保每次写论文时跑出的地图形状完全一样
    rand('state', level * 100);

    % 生成随机散落的矩形障碍物块 (模拟论文中的紫色块)
    for k = 1:num_blocks
        % 随机生成障碍物的宽和高 (1x1 到 4x4 的各种组合)
        w = floor(rand() * 4);
        h = floor(rand() * 4);

        % 随机生成障碍物的左下角坐标
        x = floor(rand() * (map_size - w - 2)) + 2;
        y = floor(rand() * (map_size - h - 2)) + 2;

        % 填充障碍物
        map_grid(x:x+w, y:y+h) = 1;
    end

    % 强制挖空起点和终点周围的安全区域，防止出生点被堵死导致无解
    safe_zone = ceil(map_size * 0.1); % 根据地图大小动态调整安全区
    map_grid(1:safe_zone, 1:safe_zone) = 0;
    map_grid(map_size-safe_zone+1:map_size, map_size-safe_zone+1:map_size) = 0;
end
