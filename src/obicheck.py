import cv2
import numpy as np
import random

def is_linear_arrangement_with_noise(img, threshold=100, inlier_threshold=10, ransac_prob=0.99, ransac_iters=100):
    """
    画像中の点の集合が直線上に並んでいるかをRANSACを用いて判定する関数（Python版）

    Args:
        img (numpy.ndarray): 入力画像（グレースケール）。
        threshold (int): 二値化の閾値。
        inlier_threshold (int): 直線からの距離がこの閾値以下の点をインライアとする。
        ransac_prob (float): 必要なインライアの割合。これ以上の割合のインライアがあれば、線形配置と判定。
        ransac_iters (int): RANSACの繰り返し回数。

    Returns:
        bool: 線形配置と判定された場合はTrue、そうでない場合はFalse。
    """
    try:
        # 画像が空かどうかを確認
        if img is None or img.size == 0:
            print("Error: 画像が空です。")
            return False

        # グレースケール画像であることを確認
        if len(img.shape) == 3:
            gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            gray_img = img.copy()

        # 二値化
        _, img_bin = cv2.threshold(gray_img, threshold, 255, cv2.THRESH_BINARY)

        # 輪郭抽出
        contours, _ = cv2.findContours(img_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 重心のリストを初期化
        centroids = []

        # 各輪郭に対して重心を計算
        for contour in contours:
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m01"])
                centroids.append((cX, cY))

        # 重心が2つ未満の場合は線形配置を判定できない
        if len(centroids) < 2:
            print("Error: 重心が2つ未満です。")
            return False

        # RANSAC
        inliers = []
        best_line = None

        for i in range(ransac_iters):
            # ランダムに2点を選択
            idx1 = random.randint(0, len(centroids) - 1)
            idx2 = idx1
            while idx2 == idx1:
                idx2 = random.randint(0, len(centroids) - 1)

            p1 = centroids[idx1]
            p2 = centroids[idx2]

            # 直線を定義
            vx = p2[0] - p1[0]
            vy = p2[1] - p1[1]
            norm = np.sqrt(vx * vx + vy * vy)
            vx /= norm
            vy /= norm

            # インライアを特定
            current_inliers = []
            for j in range(len(centroids)):
                dist = abs(vy * (centroids[j][0] - p1[0]) - vx * (centroids[j][1] - p1[1]))
                if dist < inlier_threshold:
                    current_inliers.append(j)

            # 現在のインライア数が以前の最大数よりも多い場合、結果を更新
            if len(current_inliers) > len(inliers):
                inliers = current_inliers

                # インライアの重心を計算
                inlier_points = []
                for idx in inliers:
                    inlier_points.append(centroids[idx])

                # 最小二乗法で直線を再フィッティング
                if len(inlier_points) > 1:
                    points = np.array(inlier_points)
                    vx, vy, x, y = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)
                    best_line = (vx, vy, x, y)  # best_lineを更新
                else:
                    best_line = None

        # RANSACの結果、十分な数のインライアがあるか確認
        if len(inliers) >= len(centroids) * ransac_prob and best_line is not None:
            return True
        else:
            return False

    except Exception as e:
        print(f"Error: 予期せぬエラーが発生しました: {e}")
        return False


if __name__ == "__main__":
    # 画像ファイルへのパス
    image_file_path1 = "/home/pi/kyouyuu/LOG/ball_rireki_2025_02_28_133400_130.jpg"  # �K�؂ȃp�X�ɒu�������Ă�������
    image_file_path2 = "/home/pi/kyouyuu/LOG/ball_rireki_2025_03_01_103024_214.jpg"  # �K�؂ȃp�X�ɒu�������Ă�������

    # 画像を読み込む
    img1 = cv2.imread(image_file_path1)
    img2 = cv2.imread(image_file_path2)

    # 線形配置であるかを判定
    is_linear1 = is_linear_arrangement_with_noise(img1)
    is_linear2 = is_linear_arrangement_with_noise(img2)

    # 結果を出力
    print(f"{image_file_path1}: 線形配置 = {is_linear1}")
    print(f"{image_file_path2}: 線形配置 = {is_linear2}")
