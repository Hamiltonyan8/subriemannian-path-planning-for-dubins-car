import matplotlib.pyplot as plt
import numpy as np

def plot_path(start_pose, end_pose, path_points, path_type, path_length, turn_radius):
    """
    計算された経路をプロットする関数
    """
    plt.figure(figsize=(10, 8))
    
    # 経路の軌跡をプロット
    if path_points is not None and len(path_points) > 1:
        plt.plot(path_points[:, 0], path_points[:, 1], 'b-', label=f"Path: {path_type}")

    # スタートとゴールの姿勢を矢印でプロット
    arrow_length = turn_radius * 0.8
    plt.arrow(start_pose[0], start_pose[1],
              arrow_length * np.cos(start_pose[2]),
              arrow_length * np.sin(start_pose[2]),
              color='g', width=0.1, head_width=0.4, label='Start Pose')

    plt.arrow(end_pose[0], end_pose[1],
              arrow_length * np.cos(end_pose[2]),
              arrow_length * np.sin(end_pose[2]),
              color='r', width=0.1, head_width=0.4, label='End Pose')

    # グラフの体裁を整える
    plt.title(f"Dubins Shortest Path\nType: {path_type}, Length: {path_length:.2f}")
    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    plt.grid(True)
    plt.axis("equal")
    plt.legend()
    plt.show()
