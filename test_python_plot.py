import serial
import matplotlib.pyplot as plt

# Connect to Arduino
ser = serial.Serial('COM9', 250000, timeout=1)

# Storage
targets = []
feedbacks = []
errors = []
pwms = []

print("📡 Reading from Arduino...")

try:
    while True:
        line = ser.readline().decode('utf-8').strip()

        if "Target" in line and "Feedback" in line:
            try:
                parts = line.split(',')
                target = int(parts[0].split(':')[1].strip())
                feedback = float(parts[1].split(':')[1].strip())
                error = float(parts[2].split(':')[1].strip())
                pwm = int(parts[3].split(':')[1].strip())

                targets.append(target)
                feedbacks.append(feedback)
                errors.append(error)
                pwms.append(pwm)

                print(f"T: {target}, F: {feedback}, E: {error}, PWM: {pwm}")

                if len(targets) >= 300:  # Collect 300 points then stop
                    break

            except Exception as e:
                print("⚠️ Parsing error:", e)
except KeyboardInterrupt:
    print("Stopped.")

# Plotting
plt.figure(figsize=(10, 6))
plt.plot(targets, label='Target')
plt.plot(feedbacks, label='Feedback')
plt.plot(errors, label='Error')
plt.xlabel('Time step')
plt.ylabel('Value')
plt.title('PID Control Performance')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
