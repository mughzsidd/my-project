import serial
import matplotlib.pyplot as plt
from collections import deque

# === Setup Serial ===
# Ensure your STM32 code is also updated to 115200 in MX_USART1_UART_Init
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
plt.ion()

# Data buffers (Updated to match main.c output)
angle_vals = deque(maxlen=500)      # shared_angle
acc_angle_vals = deque(maxlen=500)  # acc_angle
gyro_rate_vals = deque(maxlen=500)  # gyro_rate
pid_out_vals = deque(maxlen=500)    # shared_pid_out
time_ms = deque(maxlen=500)
cnt = 0

# === Plotting Function ===
def makeFig():
    plt.clf() # Clear entire figure for fresh subplots
    
    # Subplot 1: Angles (Filtered vs Accelerometer)
    plt.subplot(2, 1, 1)
    plt.title('Robot Orientation (Degrees)')
    plt.grid(True)
    if len(angle_vals) > 0:
        plt.plot(list(time_ms), list(angle_vals), 'm-', label='Filtered Angle', linewidth=2)
        plt.plot(list(time_ms), list(acc_angle_vals), 'r-', label='Raw Acc Angle', alpha=0.3, markersize=2)
    plt.ylabel('Angle (deg)')
    plt.legend(loc='upper right')
    plt.ylim(-45, 45) # Adjusted for balancing range
    
    # Subplot 2: PID Output
    plt.subplot(2, 1, 2)
    plt.title('PID Controller Output (Motor Command)')
    plt.grid(True)
    if len(pid_out_vals) > 0:
        plt.plot(list(time_ms), list(pid_out_vals), 'b-', label='PID Out')
    plt.xlabel('Sample Count')
    plt.ylabel('PWM Value')
    plt.legend(loc='upper right')
    plt.ylim(-1000, 1000) # Matches your PID_OUT_MAX
    
    plt.tight_layout()

# === Main Loop ===
print("Starting Plotter at 115200 Baud...")
print("Receiving: Filtered Angle, Acc Angle, Gyro Rate, PID Output")

try:
    while True:
        if ser.inWaiting() > 0:
            try:
                line = ser.readline().decode('utf-8').strip()
                values = line.split(',')

                # Match the 4 values sent by your sprintf in main.c
                if len(values) == 4:
                    angle_vals.append(float(values[0]))
                    acc_angle_vals.append(float(values[1]))
                    gyro_rate_vals.append(float(values[2]))
                    pid_out_vals.append(float(values[3]))
                    
                    time_ms.append(cnt)
                    cnt += 1
                    
                    # Performance optimization: Update plot every 10 samples
                    if cnt % 10 == 0:
                        makeFig()
                        plt.pause(0.001)

            except (ValueError, UnicodeDecodeError):
                # Ignore partial lines or decoding errors
                continue

except KeyboardInterrupt:
    print("\nStopping...")
finally:
    ser.close()
    plt.close()