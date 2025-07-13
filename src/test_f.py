import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.animation as animation
import pandas as pd
import tkinter as tk
from tkinter import filedialog

# グローバル変数でデータ保持
global df, sc, ax, shadow

def get_ball_from_csv(csv_file):
    df = pd.read_csv(csv_file)
    
    if 'camera_no' not in df.columns:
        df['camera_no'] = 1
        
    if 'radius' not in df.columns:
        df['radius'] = 5
        
    return df

def select_csv_file():
    root = tk.Tk()
    root.withdraw()
    return filedialog.askopenfilename(title="CSVファイルを選択", filetypes=[("CSVファイル", "*raw*.csv")])

def reload_data():
    global df, sc, ax, shadow
    # 既存の散布図と影を削除
    sc.remove()
    if 'shadow' in globals():
        shadow.remove()
    
    # 新しいCSVファイルを選択
    new_csv_file = select_csv_file()
    if not new_csv_file:
        return
    
    # データ再読み込み
    df = get_ball_from_csv(new_csv_file)
    points = df[['x', 'y', 'z']].values
    camera_nos = df['camera_no'].values
    radii = df['radius'].values
    
    # 新しいプロットを作成
    colors = ['red' if cam == 1 else 'green' for cam in camera_nos]
    sizes = 2 + radii * 50
    
    sc = ax.scatter(
        points[:, 0], 
        points[:, 1], 
        points[:, 2], 
        s=sizes,
        c=colors,
        alpha=0.7,
        edgecolors='black'
    )

    # 地面に影を描画
    shadow = ax.scatter(
        points[:, 0],
        points[:, 1],
        np.zeros_like(points[:, 2]),
        s=sizes,
        c='gray',
        alpha=0.3,
        edgecolors='none'
    )
    
    # 再描画
    plt.draw()
    record_text.set_text(f"Records: {len(df)}")

def on_key(event):
    if event.key == 'x':
        reload_data()

# 初期ファイル選択
csv_file = select_csv_file()
if not csv_file:
    exit()

# データ初期読み込み
df = get_ball_from_csv(csv_file)
points = df[['x', 'y', 'z']].values
camera_nos = df['camera_no'].values
radii = df['radius'].values

fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection='3d')

# キーボードイベントのバインド
fig.canvas.mpl_connect('key_press_event', on_key)

# 初期プロット設定
colors = ['red' if cam == 1 else 'green' for cam in camera_nos]
sizes = 2 + radii * 50

ax.set_xlim3d(-100, 100)
ax.set_ylim3d(150, 350)
ax.set_zlim3d(0, 200)
ax.set_xlabel('w')
ax.set_ylabel('h')
ax.set_zlabel('d')

strike_vertices = [
    [[21.6, 220, 100], [21.6, 220, 40], [21.6, 240, 40], [21.6, 240, 100]],
    [[-21.6, 220, 100], [-21.6, 220, 40], [-21.6, 240, 40], [-21.6, 240, 100]],
    [[21.6, 240, 100], [21.6, 240, 40], [-21.6, 240, 40], [-21.6, 240, 100]],
    [[21.6, 220, 100], [21.6, 220, 40], [0.0, 200, 40], [0.0, 200, 100]],
    [[-21.6, 220, 100], [-21.6, 220, 40], [0.0, 200, 40], [0.0, 200, 100]]
]

polygon_color = "green"
polygon_alpha = 0.5

for face in strike_vertices:
    poly = Poly3DCollection([face], alpha=polygon_alpha, facecolors=polygon_color, edgecolors='k')
    ax.add_collection3d(poly)

home_base = np.array([
    [0, 200, 0],
    [21.6, 220, 0],
    [21.6, 240, 0],
    [-21.6, 240, 0],
    [-21.6, 220, 0],
    [0, 200, 0],
])

home_base_color = "white"
home_base_polygon = Poly3DCollection([home_base], alpha=1.0, facecolors=home_base_color, edgecolors='k')
ax.add_collection3d(home_base_polygon)

record_text = ax.text2D(
    0.05, 0.95,
    f"Records: {len(df)}",
    transform=ax.transAxes,
    fontsize=12,
    color='black'
)

# ベースラインやバッターボックスの描画
ax.plot([-60, -100], [260, 300], [0, 0], color='black')
ax.plot([60, 100], [260, 300], [0, 0], color='black')
ax.plot([36, 36], [150, 260], [0, 0], color='black')
ax.plot([-36, -36], [150, 260], [0, 0], color='black')
ax.plot([36, 100], [260, 260], [0, 0], color='black')
ax.plot([36, 100], [150, 150], [0, 0], color='black')
ax.plot([-36, -100], [260, 260], [0, 0], color='black')
ax.plot([-36, -100], [150, 150], [0, 0], color='black')

# ボール描画
sc = ax.scatter(
    points[:, 0], 
    points[:, 1], 
    points[:, 2], 
    s=sizes,
    c=colors,
    alpha=0.7,
    edgecolors='black'
)

# 地面に影を描画
shadow = ax.scatter(
    points[:, 0],
    points[:, 1],
    np.zeros_like(points[:, 2]),
    s=sizes,
    c='gray',
    alpha=0.3,
    edgecolors='none'
)

def update_view(frame):
    elev = 30 - (frame % 30)
    azim = 180 + frame
    ax.view_init(elev=elev, azim=azim)
    return ax,

# アニメーション例（必要なら有効に）
# ani = animation.FuncAnimation(fig, update_view, frames=np.arange(45, 135, 1), interval=60)
# ani.save('camera_rotation.gif', writer='pillow', fps=20)

plt.show()
