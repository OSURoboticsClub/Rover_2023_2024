import matplotlib.pyplot as plt
import math
temperature = []
humidity = []
hydrogen = []
ozone = []
time = []
with open("/home/makemorerobot/Rover_2023_2024/software/ros_packages/rover2_odometry/rover2_odometry/sensor_data.csv") as file:
    i = 1
    for line in file:
        nums = line.split(",")
        temperature.append(nums[0])
        humidity.append(nums[1])
        hydrogen.append(nums[2])
        ozone.append(nums[3])
        time.append(i)

        i+=1
plt.figure()
plt.plot(time,temperature)
plt.title("Temperature vs Time")
plt.ylabel("Temperature (Celcius)")
plt.xlabel("Time (seconds)")
plt.savefig("graphs/tempPlot.png")

plt.figure()
plt.plot(time,humidity)
plt.title("Humidity vs Time")
plt.ylabel("Humidity (%)")
plt.xlabel("Time (seconds)")
plt.savefig("graphs/humidPlot.png")

plt.figure()
plt.plot(time,hydrogen)
plt.title("Hydrogen vs Time")
plt.ylabel("Hydrogen (ppm)")
plt.xlabel("Time (seconds)")
plt.savefig("graphs/hydroPlot.png")

plt.figure()
plt.plot(time,ozone)
plt.title("Ozone vs Time")
plt.ylabel("Ozone (ppb)")
plt.xlabel("Time (seconds)")
plt.savefig("graphs/ozonePlot.png")
