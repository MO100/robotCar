import time
from time import sleep
import robot_module as robot
import matplotlib.pyplot as plt
from matplotlib import animation
import threading
import math
from collections import deque

# Initialize Sensors
robot.MPU_Init()
ads = robot.ADS7830()

# Setup interactive mode
fig = plt.figure(figsize=(12, 6))

# Subplot 1: Distance vs Time
ax1 = fig.add_subplot(1, 2, 1)
time_buffer = deque(maxlen=100)
distance_buffer = deque(maxlen=100)
line, = ax1.plot([], [], 'b-')
ax1.set_title("Ultrasonic Distance (Live)")
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Distance (cm)")
ax1.set_ylim(0, 100)
ax1.grid(True)

# Subplot 2: Room Map
ax2 = fig.add_subplot(1, 2, 2)
ax2.set_title("Room Map (Live)")
ax2.set_xlim(-200, 200)
ax2.set_ylim(-200, 200)
ax2.set_aspect('equal')
ax2.grid(True)

# Robot position tracking
robot_x, robot_y = 0, 0
robot_theta = 0  # Facing right (0 degrees)
robot_trail = deque(maxlen=100)  # Store past positions
robot_arrow = ax2.arrow(0, 0, 10, 0, head_width=10, head_length=15, fc='r', ec='r')
obstacles = []

# Shared variables
distance = 0
running = True
start_time = time.time()

# Update plot function for FuncAnimation
def update_plots(frame):
    global robot_x, robot_y, robot_theta

    # Update distance plot
    current_time = time.time() - start_time
    time_buffer.append(current_time)
    distance_buffer.append(distance)
    line.set_data(time_buffer, distance_buffer)
    ax1.set_xlim(max(0, current_time - 5), current_time)

    # Update room map
    robot_trail.append((robot_x, robot_y))
    ax2.clear()
    ax2.set_title("Room Map (Live)")
    ax2.set_xlim(-200, 200)
    ax2.set_ylim(-200, 200)
    ax2.grid(True)

    if len(robot_trail) > 1:
        trail_x, trail_y = zip(*robot_trail)
        ax2.plot(trail_x, trail_y, 'g-', alpha=0.5)

    arrow_length = 20
    end_x = robot_x + arrow_length * math.cos(math.radians(robot_theta))
    end_y = robot_y + arrow_length * math.sin(math.radians(robot_theta))
    ax2.arrow(robot_x, robot_y,
              end_x - robot_x, end_y - robot_y,
              head_width=10, head_length=15, fc='r', ec='r')

    if distance < 100:
        obstacle_x = robot_x + distance * math.cos(math.radians(robot_theta))
        obstacle_y = robot_y + distance * math.sin(math.radians(robot_theta))
        ax2.plot(obstacle_x, obstacle_y, 'ro')

# Sensor loop that collects sensor data
def sensor_loop():
    global robot_x, robot_y, robot_theta, distance, running
    
    try:
        while running:
            # Get sensor data
            distance = robot.get_distance()
            print(f"Distance: {distance} cm")
            
            # Get orientation (simplified)
            acc_x = robot.read_raw_data(robot.ACCEL_XOUT_H) /16384
            acc_y = robot.read_raw_data(robot.ACCEL_YOUT_H) /16384
            acc_z = robot.read_raw_data(robot.ACCEL_ZOUT_H) /16384
            
            G_x = robot.read_raw_data(robot.GYRO_XOUT_H) /16384
            G_y = robot.read_raw_data(robot.GYRO_YOUT_H) /16384
            G_z = robot.read_raw_data(robot.GYRO_ZOUT_H) /16384
            pitch = math.degrees(math.atan2(acc_x, math.sqrt(acc_y**2 + acc_z**2)))
            roll  = math.degrees(math.atan2(acc_y, math.sqrt(acc_x**2 + acc_z**2)))

            print(f"Pitch: {pitch:.2f}°, Roll: {roll:.2f}°")
            
            # Simple movement simulation
            adc0 = ads.read_adc(0)
            adc1 = ads.read_adc(1)
            
            if adc0 > 240:
                robot.forward()
                print("Moving Forward")
                # Update position based on movement
                move_distance = 5  # cm per step
                robot_x += move_distance * math.cos(math.radians(robot_theta))
                robot_y += move_distance * math.sin(math.radians(robot_theta)) 
            else:
                robot.stop()
                # Simulate turning
                robot_theta = (robot_theta + 5) % 360

            sleep(0.1)
            
    except KeyboardInterrupt:
        print("Sensor loop interrupted.")
        running = False
        robot.stop()
        robot.cleanup()

# Start sensor thread
sensor_thread = threading.Thread(target=sensor_loop)
sensor_thread.start()

# Create the animation object (persist it)
ani = animation.FuncAnimation(fig, update_plots, interval=100)

# Show the plot and start the animation (only once)
try:
    plt.tight_layout()
    plt.show()  # Show the plot and start the animation
except KeyboardInterrupt:
    print("Visualization interrupted.")
finally:
    # Do not exit prematurely
    print("Program running. Press Ctrl+C to exit.")
    while running:
        sleep(1)  # Keep the program alive, allowing it to update the plot
    sensor_thread.join()
    plt.close()
    print("Program exited cleanly")

