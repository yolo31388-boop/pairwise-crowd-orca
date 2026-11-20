"""Core helpers for crowd avoidance.

Each function here is a small, deterministic building block used by the
crowd simulation.  They are intentionally free of hidden global state so the
same inputs always produce the same outputs across platforms and replays.
"""

import math

_EPS = 1e-9


def orca_time_horizon(relative_speed, base):
    """自适应时间窗。

    时间窗随相对速度自适应：两者接近得越快，窗口越短，避免把远处的
    相对运动当成眼前的冲突（同时让快速角色在窗口内被看到）。窗口同时
    作为拥堵传播的半平面时长，因此保持与相对速度成反比。
    """
    return adaptive_window(relative_speed, base)


def obstacle_radius(bounds):
    """障碍真实半径。

    ``bounds`` 可以是 ``(min_x, min_y, min_z, max_x, max_y, max_z)`` 或
    二维的 ``(min_x, min_y, max_x, max_y)``。返回包围盒的**半对角线**
    长度，即能把整个盒体包住的最小球的半径，而不是整条对角线（那样会
    让角色贴着柱子走时被过度推开）。
    """
    values = tuple(float(v) for v in bounds)
    if len(values) == 6:
        min_x, min_y, min_z, max_x, max_y, max_z = values
        half = ((max_x - min_x) / 2.0,
                (max_y - min_y) / 2.0,
                (max_z - min_z) / 2.0)
    elif len(values) == 4:
        min_x, min_y, max_x, max_y = values
        half = ((max_x - min_x) / 2.0,
                (max_y - min_y) / 2.0)
    else:
        raise ValueError("bounds must have 4 or 6 components")
    return math.sqrt(sum(component * component for component in half))


def shared_goal(nav_goal, local_goal, nav_valid):
    """共享目标点。

    局部避障与导航网格使用同一个目标：导航目标始终是权威目标（即使
    出口落在网格外），只有导航目标缺失时才退回局部目标，避免两套目标
    点导致人群原地打转。
    """
    if nav_goal is None:
        return local_goal
    return nav_goal


def adaptive_window(relative_speed, base):
    """避障窗口。

    基础窗口按相对速度缩放：``base / max(relative_speed, eps)``。相对
    速度为零时没有逼近趋势，直接使用基础窗口。
    """
    speed = abs(float(relative_speed))
    base = abs(float(base))
    if speed <= _EPS:
        return base
    return base / speed


def stable_key(entity_id, spawn_index):
    """确定性键。

    使用 ``(实体 id, 出生序号)`` 组成的稳定字符串键；不依赖哈希顺序，
    因此跨平台与回放都能得到相同排序。
    """
    return f"{entity_id}:{spawn_index}"


def density_limit(count, capacity):
    """密度限制。

    超过容量时只做速度/数量上限，不删除任何人（删除门口外侧的人会
    让内侧出现真空）。
    """
    count = int(count)
    capacity = int(capacity)
    if capacity < 0:
        capacity = 0
    return min(count, capacity)


def knockback_clear(velocity):
    """击退清速度。

    被击退时把避障速度清零，恢复后不会沿旧方向冲出去。
    """
    return (0.0, 0.0)


def partition_force(forces):
    """分区边界合力。

    多线程分区求解时，边界角色可能收到两份相反推力；这里对同一角色
    的候选力取平均，得到统一解，而不是让两份力互相抵消或叠加。
    """
    forces = list(forces)
    if not forces:
        return (0.0, 0.0)
    sum_x = 0.0
    sum_y = 0.0
    for fx, fy in forces:
        sum_x += float(fx)
        sum_y += float(fy)
    n = float(len(forces))
    return (sum_x / n, sum_y / n)


def slope_speed(speed, slope_deg):
    """坡道速度。

    速度按地面切向分量计算：``speed * cos(slope)``。下坡时不会把
    水平面速度当成实际速度而冲进人群。
    """
    angle = math.radians(float(slope_deg))
    return abs(float(speed)) * math.cos(angle)


def predict_position(position, velocity, accel, dt):
    """预测位置。

    匀加速模型 ``p + v*dt + 0.5*a*dt^2``；``accel`` 是其他角色带来的
    加速度，而不是只考虑静态障碍。
    """
    px, py = float(position[0]), float(position[1])
    vx, vy = float(velocity[0]), float(velocity[1])
    ax, ay = float(accel[0]), float(accel[1])
    dt = float(dt)
    half_dt2 = 0.5 * dt * dt
    return (px + vx * dt + ax * half_dt2,
            py + vy * dt + ay * half_dt2)


def neighbor_weight(distance, radius):
    """邻居权重。

    线性衰减：距离越近权重越高，超出半径权重为 0。
    """
    distance = abs(float(distance))
    radius = abs(float(radius))
    if radius <= _EPS:
        return 0.0
    weight = 1.0 - distance / radius
    return weight if weight > 0.0 else 0.0


def goal_reached(position, goal, radius):
    """目标到达。"""
    dx = float(goal[0]) - float(position[0])
    dy = float(goal[1]) - float(position[1])
    return math.hypot(dx, dy) <= float(radius)


def avoidance_enabled(count, threshold):
    """避障开关。"""
    return int(count) >= int(threshold)


def door_capacity(width, agent_radius):
    """门口容量。

    门口同一时刻能并排通过的人数：``floor(width / (2 * radius))``。
    """
    width = float(width)
    agent_radius = float(agent_radius)
    if agent_radius <= _EPS:
        return 0
    return int(math.floor(max(width, 0.0) / (2.0 * agent_radius)))


def velocity_clamp(vx, vy, max_speed):
    """速度钳制。

    超过最大速度时按比例缩放，否则原样返回。
    """
    vx = float(vx)
    vy = float(vy)
    max_speed = float(max_speed)
    speed = math.hypot(vx, vy)
    if max_speed <= 0.0:
        return (0.0, 0.0)
    if speed <= max_speed or speed <= _EPS:
        return (vx, vy)
    scale = max_speed / speed
    return (vx * scale, vy * scale)
