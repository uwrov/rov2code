import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
import glob
from matplotlib.widgets import CheckButtons

def main():
    user_input = input("Enter log CSV filename (or press Enter for most recent): ").strip()
    plt.rc('font', size = 8)
    if user_input:
        filename = user_input
    else:
        filename = find_latest_log()
        if not filename:
            print("No log files found.")
            return
        print(f"Using most recent log: {filename}")

    df = pd.read_csv(filename)
    df['time'] -= df['time'].iloc[0]

    # — outlier removal, invert & (temporary) scaling ——
    df['ax'] = remove_outliers(df['ax'])
    df['ay'] = remove_outliers(df['ay'])
    df['az'] = remove_outliers(df['az'])
    df['gx'] *= -1  # inverted

    # temporary scaling to get everything in roughly the same range
    df['cmd_fx'] *= 0.1
    df['cmd_fy'] *= 0.1
    df['cmd_fz'] *= 0.1
    df['cmd_tx'] *= 3
    df['cmd_ty'] *= 3
    df['cmd_tz'] *= 3

    # — compute induced-angular-accel errors ——
    df['roll_err']  = df['gy'] - df['cmd_ty']
    df['pitch_err'] = df['gx'] - df['cmd_tx']
    df['yaw_err']   = df['gz'] - df['cmd_tz']

    # — plot thrust, linear accel & angular-error accel ——
    fig, axs = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    translations = [
        ('cmd_fx', 'ax', '+X (right)'),
        ('cmd_fy', 'ay', '+Y (back)'),
        ('cmd_fz', 'az', '+Z (up)')
    ]
    errors = [
        ('roll_err',  'induced roll'),
        ('pitch_err', 'induced pitch'),
        ('yaw_err',   'induced yaw')
    ]

    cmd_color    = '#202020'
    lin_color    = '#AAAA00'
    error_colors = {
        'roll_err':  '#AA0000',
        'pitch_err': '#00AA00',
        'yaw_err':   '#0000AA'
    }
    lin_lines = [] 
    for ax, (cmd_col, lin_col, title) in zip(axs, translations):
        # primary y-axis: thrust cmd & linear accel
        ax.plot(df['time'], df[cmd_col], '-', color=cmd_color,  label='thrust cmd')
        ln, = ax.plot(df['time'], df[lin_col],  '-', color=lin_color,  label=f'{lin_col}')
        lin_lines.append(ln)
        ax.set_ylabel(title)
        ax.legend(loc='upper left')
        ax.grid(True)

        # secondary y-axis: angular-error accel
        ax2 = ax.twinx()
        for err_col, err_label in errors:
            ax2.plot(df['time'], df[err_col], '--', color=error_colors[err_col], label=err_label)
        ax2.set_ylabel('ang-accel err')
        ax2.legend(loc='upper right')

    axs[-1].set_xlabel('Time [s]')


    # — simple cross-correlation analysis ——
    dt = df['time'].diff().mean()
    print("\nCoupling analysis (lag in s & normalized peak corr):")
    for cmd_col, _, _ in translations:
        for err_col, _ in errors:
            x = df[cmd_col] - df[cmd_col].mean()
            y = df[err_col] - df[err_col].mean()
            corr = np.correlate(x, y, mode='full')
            lags = np.arange(-len(x)+1, len(x))
            idx = corr.argmax()
            lag_s = lags[idx] * dt
            # normalize by N·σx·σy
            strength = corr[idx] / (len(x) * df[cmd_col].std() * df[err_col].std())
            print(f"  {cmd_col:7s} → {err_col:8s}: lag={lag_s:6.3f}s, corr={strength:5.3f}")

    plt.tight_layout()

    def toggle_lin(label):
        for ln in lin_lines:
            ln.set_visible(not ln.get_visible())
        plt.draw()

    # rax = plt.axes([0.88, 0.4, 0.1, 0.05])    # x0, y0, width, height
    # check = CheckButtons(rax, ['lin accel'], [True])
    # check.on_clicked(toggle_lin)

    plt.show()

    

def remove_outliers(series, z_thresh=4):
    m, s = series.mean(), series.std()
    return series.where(abs(series - m) < z_thresh * s)

def find_latest_log(log_dir="C:/Users/uwrov/Documents/GitHub/rovcode/surface/logs"):
    logs = glob.glob(os.path.join(log_dir, "log_*.csv"))
    return max(logs, key=os.path.getmtime) if logs else None

if __name__ == "__main__":
    main()
