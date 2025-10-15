import numpy as np
from dubins_path import find_shortest_path
from simulation import plot_path

def main():
    """
    メイン実行関数
    """
    # --- 問題設定 ---
    # スタート姿勢 (x, y, theta)
    start_pose = (0.0, 0.0, np.deg2rad(45))
    
    # ゴール姿勢 (x, y, theta)
    end_pose = (10.0, 5.0, np.deg2rad(-30))
    
    # ロボットの最小回転半径
    turn_radius = 2.0
    
    print("Calculating shortest Dubins path...")
    print(f"Start: {start_pose}")
    print(f"End:   {end_pose}")
    print(f"Turn Radius: {turn_radius}")
    
    # --- 経路計算 ---
    best_path_type, path_length, path_points = find_shortest_path(start_pose, end_pose, turn_radius)
    
    # --- 結果の表示とプロット ---
    if best_path_type:
        print(f"\nFound shortest path!")
        print(f"Path Type: {best_path_type}")
        print(f"Path Length: {path_length:.2f}")
        
        plot_path(start_pose, end_pose, path_points, best_path_type, path_length, turn_radius)
    else:
        print("\nCould not find a valid path.")


if __name__ == "__main__":
    main()
