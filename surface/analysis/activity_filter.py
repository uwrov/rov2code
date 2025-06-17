import pandas
import os

FOLDER_PATH = "../logs/"
TIME_COL = "time"
CMD_COLS = ["cmd_fx", "cmd_fy", "cmd_fz", "cmd_tx", "cmd_ty", "cmd_tz"]
PWM_COLS = ["pwm_1", "pwm_2", "pwm_3", "pwm_4", "pwm_5", "pwm_6"]

MIN_DUR_SEC = 0
MAX_DUR_SEC = 150

usable_files = []

for filename in os.listdir(FOLDER_PATH):
    if filename.endswith(".csv"):
        file_path = os.path.join(FOLDER_PATH, filename)
        try:
            df = pandas.read_csv(file_path)

            if df.empty:
                print(f"Skipping {filename}: file is empty")
                continue

            if TIME_COL not in df.columns or not all(col in df.columns for col in CMD_COLS or PWM_COLS):
                print(f"Skipping {filename}: missing column(s)")

            dur_sec = df[TIME_COL].iloc[-1] - df[TIME_COL].iloc[0]
            if not (MIN_DUR_SEC <= dur_sec <= MAX_DUR_SEC):
                continue

            cmd_data = df[CMD_COLS].fillna(0)
            thrust_rows = (cmd_data != 0).any(axis=1)
            thrust_cnt = thrust_rows.sum()

            if thrust_cnt > 20:
                usable_files.append(filename)

        except Exception as e:
            print(f"Error reading {filename}: {e}")

print("Usable session files:")
for f in usable_files:
    print(f)

            
