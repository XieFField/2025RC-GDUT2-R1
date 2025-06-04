import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
import matplotlib

matplotlib.rcParams['mathtext.default'] = 'regular'
matplotlib.rcParams['font.sans-serif'] = ['SimHei']  # 中文字体
matplotlib.rcParams['axes.unicode_minus'] = False


def calculate_r_squared(y_true, y_pred):
    """计算R²值"""
    ss_res = np.sum((y_true - y_pred) ** 2)
    ss_tot = np.sum((y_true - np.mean(y_true)) ** 2)
    return 1 - (ss_res / ss_tot)


def create_friction_wheel_fitting():
    """
    使用三次样条插值分析机器人与篮框距离和摩擦轮转速的关系
    """

    print("=== 摩擦轮转速拟合程序 ===")
    print("请输入10组数据：距离(m) 转速(rpm)")
    print("格式：距离 转速（用空格分隔）")
    print("例如：1.5 3000")
    print("-" * 40)

    distances = []  # 距离数据
    speeds = []  # 转速数据

    # 输入数据
    for i in range(10):
        while True:
            try:
                user_input = input(f"第{i + 1}组数据: ").strip()
                distance, speed = map(float, user_input.split())
                distances.append(distance)
                speeds.append(speed)
                break
            except ValueError:
                print("输入格式错误，请重新输入！格式：距离 转速")

    # 转换为numpy数组并排序
    distances = np.array(distances)
    speeds = np.array(speeds)

    # 按距离排序
    sorted_indices = np.argsort(distances)
    distances_sorted = distances[sorted_indices]
    speeds_sorted = speeds[sorted_indices]

    print("\n=== 数据汇总 ===")
    for i, (d, s) in enumerate(zip(distances_sorted, speeds_sorted)):
        print(f"距离: {d:.2f}m, 转速: {s:.0f}rpm")

    # 生成平滑曲线的点
    distance_range = np.linspace(distances_sorted.min(), distances_sorted.max(), 300)

    # 三次样条插值拟合
    cs = CubicSpline(distances_sorted, speeds_sorted)
    spline_r2 = calculate_r_squared(speeds_sorted, cs(distances_sorted))

    print(f"\n=== 三次样条插值拟合结果 ===")
    print(f"拟合优度 R² = {spline_r2:.6f}")

    # 提取每段样条的系数
    coefficients = cs.c  # 样条系数，形状为 (4, n-1)，分别对应 x³, x², x, 常数项
    segments = cs.x  # 样条分段的节点

    print("\n=== 三次样条插值分段公式 ===")
    for i in range(len(segments) - 1):
        a, b, c, d = coefficients[:, i]
        print(f"区间 [{segments[i]:.2f}, {segments[i+1]:.2f}]:")
        print(f"f(x) = {a:.6f} * (x - {segments[i]:.2f})³ + {b:.6f} * (x - {segments[i]:.2f})² + {c:.6f} * (x - {segments[i]:.2f}) + {d:.6f}")

    # 绘制图形
    plt.figure(figsize=(12, 8))

    # 原始数据点
    plt.scatter(distances_sorted, speeds_sorted, color='red', s=100,
                label='实测数据点', zorder=5, edgecolor='black', linewidth=1)

    # 三次样条插值曲线
    plt.plot(distance_range, cs(distance_range), 'b-', linewidth=3,
             label=f'三次样条插值拟合曲线 (R² = {spline_r2:.4f})')

    # 添加数据点标注
    for i, (d, s) in enumerate(zip(distances_sorted, speeds_sorted)):
        plt.annotate(f'({d:.1f}, {s:.0f})',
                     (d, s),
                     xytext=(5, 5),
                     textcoords='offset points',
                     fontsize=9,
                     bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.7))

    plt.xlabel('距离 (m)', fontsize=12)
    plt.ylabel('摩擦轮转速 (rpm)', fontsize=12)
    plt.title('机器人与篮框距离 vs 摩擦轮转速三次样条插值拟合曲线', fontsize=14)
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize=11)

    # 设置坐标轴范围
    x_margin = (distances_sorted.max() - distances_sorted.min()) * 0.1
    y_margin = (speeds_sorted.max() - speeds_sorted.min()) * 0.1
    plt.xlim(distances_sorted.min() - x_margin, distances_sorted.max() + x_margin)
    plt.ylim(speeds_sorted.min() - y_margin, speeds_sorted.max() + y_margin)

    plt.tight_layout()
    plt.show()

    # 提供查询功能
    print(f"\n=== 转速查询 ===")
    while True:
        try:
            query_distance = input("\n输入距离查询对应转速(输入'q'退出): ")
            if query_distance.lower() == 'q':
                break

            query_distance = float(query_distance)
            predicted_speed = cs(query_distance)

            print(f"距离 {query_distance:.2f}m 对应的推荐转速: {predicted_speed:.0f}rpm")

        except ValueError:
            print("输入错误，请输入有效数字！")

    return cs


# 运行程序
if __name__ == "__main__":
    result = create_friction_wheel_fitting()