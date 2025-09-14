import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
import pandas as pd
import re


def input_data():
    """获取用户输入的数据集（支持批量粘贴，兼容全角/半角逗号、各类空白字符）"""
    print("请粘贴数据集（每行一个数据点，格式如 '1.19, 35500' 或 '1.40 38200'），输入空行结束：")

    data = []
    while True:
        line = input().strip()
        if not line:
            break

        # 关键处理：兼容全角逗号（，）、半角逗号（,）、中文/英文空格、制表符等
        # 用正则替换所有分隔符为半角空格
        line = re.sub(r'[，,\s　\t]+', ' ', line).strip()

        parts = line.split()

        if len(parts) != 2:
            print(f"跳过无效行：'{line}'（需为 距离 射速 格式，用空格/逗号分隔）")
            continue

        try:
            distance = float(parts[0])
            velocity = float(parts[1])
            data.append((distance, velocity))
            print(f"已添加有效数据：距离={distance}, 射速={velocity}")
        except ValueError:
            print(f"跳过无效行：'{line}'（无法转换为数字）")

    if not data:
        print("未检测到有效数据，程序退出")
        exit()

    # 按距离升序排序（保证插值正确性）
    data.sort()
    distances = np.array([d[0] for d in data])
    velocities = np.array([d[1] for d in data])

    return distances, velocities


def cubic_spline_interpolation(x, y):
    """执行三次样条插值，返回插值函数和密集点"""
    cs = CubicSpline(x, y)
    # 生成 1000 个密集点用于绘制平滑曲线
    x_interp = np.linspace(min(x), max(x), 1000)
    y_interp = cs(x_interp)
    return cs, x_interp, y_interp


def print_spline_functions(cs, x_original):
    """打印每个区间的三次样条函数表达式（三次多项式）"""
    print("\n===== 各区间拟合函数（三次多项式） =====")
    coefficients = cs.c  # 系数矩阵：[a, b, c, d] 对应 a*x³ + b*x² + c*x + d
    for i in range(len(x_original) - 1):
        a = coefficients[0, i]
        b = coefficients[1, i]
        c = coefficients[2, i]
        d = coefficients[3, i]
        x_start = x_original[i]
        x_end = x_original[i + 1]
        print(f"区间 [{x_start:.2f}, {x_end:.2f}]：")
        print(f"f(x) = {a:.4f}x³ + {b:.4f}x² + {c:.4f}x + {d:.4f}\n")


def visualize_results(x_original, y_original, x_interp, y_interp):
    """可视化原始数据点和插值曲线"""
    plt.figure(figsize=(10, 6), dpi=100)
    plt.scatter(x_original, y_original, color='red', s=60, label='原始数据点', zorder=3)
    plt.plot(x_interp, y_interp, color='blue', linewidth=2, label='三次样条插值曲线', zorder=2)

    plt.xlabel('距离', fontsize=12)
    plt.ylabel('射速', fontsize=12)
    plt.title('距离与射速关系拟合曲线', fontsize=14)
    plt.grid(True, linestyle='--', alpha=0.7, zorder=1)
    plt.legend(fontsize=10)
    plt.tight_layout()
    plt.show()


def predict_velocity(cs, x_original):
    """交互式预测指定距离的射速"""
    while True:
        try:
            user_input = input("\n请输入要预测的距离（输入 'q' 退出）: ").strip()
            if user_input.lower() == 'q':
                break

            distance = float(user_input)
            # 检查是否在插值范围内
            if distance < min(x_original) or distance > max(x_original):
                print(f"警告：距离 {distance:.2f} 超出已知数据范围 [{min(x_original):.2f}, {max(x_original):.2f}]")
                print("结果可能存在误差，请谨慎参考")

            predicted_velocity = cs(distance)
            print(f"距离 {distance:.2f} 对应的预测射速：{predicted_velocity:.2f}")

        except ValueError:
            print("输入无效，请输入数字或 'q' 退出")


def save_results(x_original, y_original, x_interp, y_interp):
    """保存原始数据和插值结果到 CSV 文件"""
    try:
        # 原始数据
        original_df = pd.DataFrame({
            '距离': x_original,
            '射速': y_original
        })
        # 插值结果
        interp_df = pd.DataFrame({
            '距离': x_interp,
            '射速': y_interp
        })

        # 保存文件（utf-8-sig 避免中文乱码）
        original_df.to_csv('原始数据.csv', index=False, encoding='utf-8-sig')
        interp_df.to_csv('插值结果.csv', index=False, encoding='utf-8-sig')

        print("\n数据已成功保存：")
        print(" - 原始数据 → 原始数据.csv")
        print(" - 插值结果 → 插值结果.csv")

    except Exception as e:
        print(f"保存文件失败：{str(e)}")


def main():
    """主流程：输入→插值→可视化→预测→保存"""
    print("===== 三次样条插值拟合工具（距离-射速） =====")

    # 1. 输入数据（兼容全角逗号）
    x, y = input_data()

    # 检查数据量
    if len(x) < 2:
        print("至少需要 2 个数据点才能进行插值，程序退出")
        return

    # 2. 执行三次样条插值
    cs, x_interp, y_interp = cubic_spline_interpolation(x, y)

    # 3. 打印各区间拟合函数
    print_spline_functions(cs, x)

    # 4. 可视化结果
    visualize_results(x, y, x_interp, y_interp)

    # 5. 交互式预测
    predict_velocity(cs, x)

    # 6. 保存结果到 CSV
    save_results(x, y, x_interp, y_interp)


if __name__ == "__main__":
    main()