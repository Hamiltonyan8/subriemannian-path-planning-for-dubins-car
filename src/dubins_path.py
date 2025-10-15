import numpy as np

def mod2pi(theta):
    """角度を 0 から 2pi の範囲に正規化する"""
    return theta - 2 * np.pi * np.floor(theta / (2 * np.pi))

# 各経路タイプの計算（ここではRSLとRLRの2つを代表として実装）
# 実際には6種類すべてを実装します

def path_RSL(alpha, beta, d):
    sa = np.sin(alpha)
    sb = np.sin(beta)
    ca = np.cos(alpha)
    cb = np.cos(beta)
    
    tmp = 2 + d**2 - 2*(ca*cb + sa*sb)
    if tmp < 0:
        return None, None, None, False
        
    p_squared = tmp
    p = np.sqrt(p_squared)
    t = mod2pi(-alpha + np.arctan2(cb-ca, d+sa-sb))
    q = mod2pi(beta - np.arctan2(cb-ca, d+sa-sb))
    
    return t, p, q, True

def path_RLR(alpha, beta, d):
    sa = np.sin(alpha)
    sb = np.sin(beta)
    ca = np.cos(alpha)
    cb = np.cos(beta)

    tmp = (6 - d**2 + 2*(ca*cb + sa*sb)) / 8
    if abs(tmp) > 1:
        return None, None, None, False
        
    p = mod2pi(np.arccos(tmp))
    t = mod2pi(-alpha + np.arctan2(ca-cb, d-sa+sb) + p/2)
    q = mod2pi(-mod2pi(beta) + alpha -t + p)

    return t, p, q, True

# 他のパスタイプ（LSL, LSR, LRL, RSR）も同様に実装...

# パスタイプとパラメータから座標点のリストを生成する関数
def get_path_points(start_pose, path_type, params, turn_radius, step_size=0.1):
    points = [start_pose[:2]]
    
    current_pose = np.array(start_pose, dtype=float)
    
    segment_lengths = []
    if path_type == "RSL":
        segment_lengths = [params[0], params[1], params[2]]
        directions = [1, 0, -1] # 1:Right, -1:Left, 0:Straight
    elif path_type == "RLR":
        segment_lengths = [params[0], params[1], params[2]]
        directions = [1, -1, 1]
    # 他のパスタイプも同様に...
    else:
        return [start_pose[:2]] # 未実装の場合はスタート地点のみ返す

    for i in range(3):
        length = segment_lengths[i] * turn_radius
        direction = directions[i]
        
        n_steps = int(np.ceil(length / step_size))
        
        for _ in range(n_steps):
            if direction == 0: # 直進
                current_pose[0] += step_size * np.cos(current_pose[2])
                current_pose[1] += step_size * np.sin(current_pose[2])
            else: # 旋回
                omega = direction / turn_radius
                d_theta = omega * step_size
                current_pose[0] += turn_radius * (np.sin(current_pose[2] + d_theta) - np.sin(current_pose[2])) * direction
                current_pose[1] += turn_radius * (np.cos(current_pose[2]) - np.cos(current_pose[2] + d_theta)) * direction
                current_pose[2] = mod2pi(current_pose[2] + d_theta)
            
            points.append(current_pose[:2].copy())
            
    return np.array(points)


def find_shortest_path(start_pose, end_pose, turn_radius):
    """最短経路を見つけるメイン関数"""
    
    # --- 座標変換 ---
    # スタート地点を原点(0,0,0)に移動し、ゴール地点を相対的に変換
    dx = end_pose[0] - start_pose[0]
    dy = end_pose[1] - start_pose[1]
    d = np.sqrt(dx**2 + dy**2) / turn_radius
    
    theta = mod2pi(np.arctan2(dy, dx))
    alpha = mod2pi(start_pose[2] - theta)
    beta = mod2pi(end_pose[2] - theta)
    
    # --- 各経路候補を計算 ---
    paths = {}
    
    # ここでは代表として2つだけ計算
    t, p, q, is_valid = path_RSL(alpha, beta, d)
    if is_valid:
        paths["RSL"] = (abs(t) + abs(p) + abs(q)) * turn_radius, ("RSL", (t, p, q))
        
    t, p, q, is_valid = path_RLR(alpha, beta, d)
    if is_valid:
        paths["RLR"] = (abs(t) + abs(p) + abs(q)) * turn_radius, ("RLR", (t, p, q))

    # 他の4パターンも同様に追加...
    
    if not paths:
        return None, None, None

    # --- 最短経路を選択 ---
    best_path_key = min(paths, key=lambda k: paths[k][0])
    best_path_type, best_params = paths[best_path_key][1]
    
    # --- 軌跡を生成 ---
    path_points = get_path_points(start_pose, best_path_type, best_params, turn_radius)
    
    return best_path_type, paths[best_path_key][0], path_points
