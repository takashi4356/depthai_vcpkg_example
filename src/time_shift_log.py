from datetime import datetime, timedelta

def shift_log_timestamps(input_file, output_file, shift_seconds):
    with open(input_file, 'r', encoding='utf-8-sig') as infile, open(output_file, 'w', encoding='utf-8') as outfile:
        for line in infile:
            # タイムスタンプを抽出
            timestamp_str = line[:23]  # "YYYY-MM-DD HH:MM:SS.sss" の長さは23文字
            try:
                # タイムスタンプをdatetimeオブジェクトに変換
                timestamp = datetime.strptime(timestamp_str, '%Y-%m-%d %H:%M:%S.%f')
                # タイムスタンプをずらす
                new_timestamp = timestamp + timedelta(seconds=shift_seconds)
                # 新しいタイムスタンプを文字列に変換
                new_timestamp_str = new_timestamp.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]  # ミリ秒まで表示
                
                # 新しい行を作成
                new_line = new_timestamp_str + line[23:]  # タイムスタンプ以外の部分をそのまま追加
                outfile.write(new_line)
            except ValueError as e:
                print(f"エラーが発生しました: {e} - 行: {line.strip()}")

# 使用例
input_log_file = '/home/pi/kyouyuu/LOG/strike_log_2025_01_05.log'  # 読み込むログファイルのパス
output_log_file = '/home/pi/kyouyuu/LOG/strike_log_2025_01_05s.log'  # 書き込む新しいログファイルのパス
shift_seconds = -3  # タイムスタンプをずらす秒数

shift_log_timestamps(input_log_file, output_log_file, shift_seconds)
