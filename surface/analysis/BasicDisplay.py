import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
import glob

def find_latest_log(log_dir="C:/Users/uwrov/Documents/GitHub/rovcode/surface/logs"):
    logs = glob.glob(os.path.join(log_dir, "log_*.csv"))
    if not logs:
        return None
    return max(logs, key=os.path.getmtime)

def main():
    user_input = input("Enter log CSV filename (or press Enter for most recent): ").strip()

    if user_input:
        filename = user_input
    else:
        filename = find_latest_log()
        if not filename:
            print("No log files found.")
            return
        print(f"Using most recent log: {filename}")

    df = load_log(filename)
    plot_acceleration(df)
    plot_angular_velocity(df)
    plot_pwm(df)
    plt.show()

def load_log(filename):
    df = pd.read_csv(filename)
    df['time'] -= df['time'].iloc[0]  # Normalize time
    return df

def plot_acceleration(df):
    plt.figure()
    plt.plot(df['time'], df['ax'], label='ax')
    plt.plot(df['time'], df['ay'], label='ay')
    plt.plot(df['time'], df['az'], label='az')
    plt.title("Linear Acceleration")
    plt.xlabel("Time [s]")
    plt.ylabel("Acceleration [m/s²]")
    plt.legend()
    plt.grid()

def plot_angular_velocity(df):
    plt.figure()
    plt.plot(df['time'], df['gx'], label='gx')
    plt.plot(df['time'], df['gy'], label='gy')
    plt.plot(df['time'], df['gz'], label='gz')
    plt.title("Angular Velocity")
    plt.xlabel("Time [s]")
    plt.ylabel("Angular Velocity [rad/s]")
    plt.legend()
    plt.grid()

def plot_pwm(df):
    plt.figure()
    for i in range(1, 7):
        col = f'pwm_{i}'
        if col in df.columns:
            plt.plot(df['time'], df[col], label=col)
    plt.title("Thruster PWM Outputs")
    plt.xlabel("Time [s]")
    plt.ylabel("PWM Value")
    plt.legend()
    plt.grid()

if __name__ == "__main__":
    main()
