import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt
import os

class RCBoatAnalyzer:
    def __init__(self, filepath):
        self.df = pd.read_csv(filepath)
        self._preprocess_data()
        
    def _preprocess_data(self):
        # Convert Unix timestamp (ms) to relative seconds
        start_time = self.df['timestamp'].iloc[0]
        self.df['time_sec'] = (self.df['timestamp'] - start_time) / 1000.0
        self.df = self.df.sort_values(by='time_sec').reset_index(drop=True)
        self.df['dt'] = self.df['time_sec'].diff().fillna(0)

    def apply_lowpass_filter(self, columns, cutoff_freq=2.0, sample_rate=10.0):
        nyquist = 0.5 * sample_rate
        normal_cutoff = cutoff_freq / nyquist
        b, a = butter(2, normal_cutoff, btype='low', analog=False)
        for col in columns:
            if col in self.df.columns:
                self.df[f'{col}_smooth'] = filtfilt(b, a, self.df[col])
                
    def calculate_kinematics(self):
        # 1. High-Pass Filter on Y-Acceleration to remove Gravity Bleed
        # This ignores the slow tilt of the boat and focuses on actual thrust
        nyquist = 0.5 * 10.0 
        b_hp, a_hp = butter(1, 0.5 / nyquist, btype='high')
        self.df['accY_hp'] = filtfilt(b_hp, a_hp, self.df['accY'])
        
        # Convert Gs to m/s^2
        self.df['accY_ms2'] = self.df['accY_hp'] * 9.81
        
        # 2. Leaky Integrator Loop
        vel_ms = [0.0]
        dt_vals = self.df['dt'].values
        acc_vals = self.df['accY_ms2'].values
        
        # Leak factor combats drift. 0.98 means it forgets 2% of drift every loop
        leak_factor = 0.98 
        
        for i in range(1, len(acc_vals)):
            # Add new acceleration to the slightly decayed previous velocity
            new_v = (vel_ms[i-1] * leak_factor) + (acc_vals[i] * dt_vals[i])
            
            # Zero-Floor: Prevent massive negative drift (assumes boat mostly goes forward)
            new_v = max(0.0, new_v) 
            vel_ms.append(new_v)
            
        self.df['velY_ms'] = vel_ms
        self.df['velY_mph'] = self.df['velY_ms'] * 2.23694

        # 3. Forward Distance
        self.df['distY_m'] = (self.df['velY_ms'] * self.df['dt']).cumsum()
        
        # 4. Heading
        self.df['heading_deg'] = (self.df['gyroZ_smooth'] * self.df['dt']).cumsum()

    def get_meaningful_metrics(self):
        metrics = {}
        metrics['Max Forward Accel (G)'] = round(self.df['accY'].max(), 3)
        metrics['Max Lateral Accel (G)'] = round(abs(self.df['accX']).max(), 3)
        
        if 'velY_mph' in self.df.columns:
            metrics['Estimated Top Speed (mph)'] = round(self.df['velY_mph'].max(), 2)
            metrics['Estimated Distance Traveled (m)'] = round(self.df['distY_m'].iloc[-1], 2)
        
        metrics['Max Turn Rate (deg/s)'] = round(abs(self.df['gyroZ']).max(), 3)
        return metrics

    def plot_kinematics(self):
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        
        # Plot 1: Estimated Velocity
        ax1.plot(self.df['time_sec'], self.df['velY_mph'], color='tab:red', linewidth=2)
        ax1.set_ylabel('Velocity (MPH)')
        ax1.set_title('Estimated Forward Velocity (Drift Corrected)')
        ax1.grid(True, alpha=0.3)
        
        # Plot 2: Estimated Distance
        ax2.plot(self.df['time_sec'], self.df['distY_m'], color='tab:green', linewidth=2)
        ax2.set_ylabel('Distance (Meters)')
        ax2.set_title('Estimated Forward Distance')
        ax2.grid(True, alpha=0.3)
        
        # Plot 3: Heading
        ax3.plot(self.df['time_sec'], self.df['heading_deg'], color='tab:purple', linewidth=2)
        ax3.set_ylabel('Heading (Degrees)')
        ax3.set_xlabel('Time (Seconds)')
        ax3.set_title('Relative Boat Heading')
        ax3.grid(True, alpha=0.3)
        
        fig.tight_layout()
        plt.show()

if __name__ == "__main__":
    print("Welcome to the RC Boat Telemetry & Kinematics Analyzer!")
    user_filepath = input("Please enter the path to your CSV file: ").strip()
    user_filepath = user_filepath.strip("\"'")

    if not os.path.exists(user_filepath):
        print(f"\nError: Could not find the file at '{user_filepath}'.")
    else:
        print(f"\nLoading data from {user_filepath}...")
        try:
            analyzer = RCBoatAnalyzer(user_filepath)
            analyzer.apply_lowpass_filter(['accX', 'accY', 'accZ', 'gyroZ'], cutoff_freq=1.5, sample_rate=10.0)
            analyzer.calculate_kinematics()
            
            print("\n--- RC Boat Performance Metrics ---")
            metrics = analyzer.get_meaningful_metrics()
            for key, value in metrics.items():
                print(f"{key}: {value}")
                
            print("\nGenerating graphs... (Close the graph windows to finish the script)")
            analyzer.plot_kinematics()
            
        except Exception as e:
            print(f"\nAn error occurred while processing the file: {e}")
