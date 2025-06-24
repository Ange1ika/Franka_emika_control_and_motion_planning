import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("/home/abdallaswikir/Desktop/robot_prj/data/trajectory_log.csv")

# Plot position
plt.figure(figsize=(10, 5))
plt.plot(df["time"], df["px"], label="x")
plt.plot(df["time"], df["py"], label="y")
plt.plot(df["time"], df["pz"], label="z")
plt.xlabel("Time [s]")
plt.ylabel("Position [m]")
plt.title("End-effector Position over Time")
plt.legend()
plt.grid()

# Plot orientation
plt.figure(figsize=(10, 5))
plt.plot(df["time"], df["qx"], label="qx")
plt.plot(df["time"], df["qy"], label="qy")
plt.plot(df["time"], df["qz"], label="qz")
plt.plot(df["time"], df["qw"], label="qw")
plt.xlabel("Time [s]")
plt.ylabel("Quaternion")
plt.title("Orientation over Time")
plt.legend()
plt.grid()

plt.show()
