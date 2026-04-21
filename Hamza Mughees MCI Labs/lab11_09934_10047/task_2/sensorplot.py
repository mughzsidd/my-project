import serial
import matplotlib.pyplot as plt
from collections import deque

# === Setup Serial ===
ser = serial.Serial('/dev/ttyACM0', 115200)
plt.ion()

# Data buffers
accX_vals = deque(maxlen=500)
gyroX_vals = deque(maxlen=500)
angle_vals = deque(maxlen=500)
filtered_accX_vals = deque(maxlen=500)
time_ms = deque(maxlen=500)
cnt = 0

# Filter parameters
ALPHA = 0.3  # Smoothing factor (0-1), lower = more smoothing
filtered_acc = 0  # Initial filtered value

# === Plotting Function ===
def makeFig():
    plt.subplot(2, 1, 1)
    plt.cla()
    plt.title('Accelerometer X - Raw vs Exponential Moving Average')
    plt.grid(True)
    plt.xlabel('Sample Number')
    plt.ylabel('Acceleration (g)')
    
    if len(accX_vals) > 0:
        plt.plot(list(time_ms), list(accX_vals), 'r.-', label='Raw accX', alpha=0.4, markersize=2)
    if len(filtered_accX_vals) > 0:
        plt.plot(list(time_ms), list(filtered_accX_vals), 'b-', label='EMA Filtered', linewidth=2)
    
    plt.legend(loc='upper right')
    plt.ylim(-2, 2)
    
    plt.subplot(2, 1, 2)
    plt.cla()
    plt.title('Gyroscope X and Complementary Filter Angle')
    plt.grid(True)
    plt.xlabel('Sample Number')
    plt.ylabel('Value')
    
    if len(gyroX_vals) > 0:
        plt.plot(list(time_ms), list(gyroX_vals), 'g.-', label='Gyro X (dps)', alpha=0.4, markersize=2)
    if len(angle_vals) > 0:
        plt.plot(list(time_ms), list(angle_vals), 'm-', label='Filtered Angle (deg)', linewidth=2)
    
    plt.legend(loc='upper right')
    plt.ylim(-180, 180)
    plt.tight_layout()

# === Main Loop ===
print("Starting plot with Exponential Moving Average filter (Alpha = 0.3)")
print("Graph 1: Raw (red) vs Filtered (blue) Accelerometer X")
print("Graph 2: Gyroscope X (green) and Filtered Angle (magenta)")

try:
    while True:
        while ser.inWaiting() == 0:
            pass

        try:
            line = ser.readline().decode().strip()
            values = line.split(',')

            if len(values) == 3:
                accX = float(values[0])
                gyroX = float(values[1])
                angle = float(values[2])
                
                # Apply Exponential Moving Average (EMA) filter
                if cnt == 0:
                    filtered_acc = accX  # Initialize on first sample
                else:
                    filtered_acc = ALPHA * accX + (1 - ALPHA) * filtered_acc
                
                # Store values
                accX_vals.append(accX)
                filtered_accX_vals.append(filtered_acc)
                gyroX_vals.append(gyroX)
                angle_vals.append(angle)
                time_ms.append(cnt)
                cnt += 10
                
                makeFig()
                plt.pause(0.0001)

        except Exception as e:
            print(f"Error: {e}")
            continue

except KeyboardInterrupt:
    print("\nStopping...")
    
finally:
    ser.close()
    plt.close()
    print("Done!")