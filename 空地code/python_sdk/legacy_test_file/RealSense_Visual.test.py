import time
import threading
import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
from matplotlib import axes
from FlightController.Components.RealSense import T265, T265_Pose_Frame

pf = pd.DataFrame(columns=["x", "y", "z", "timestamp"])

lock = threading.Lock()
def callback(pose: T265_Pose_Frame, id: int, timestamp: float):
    global pf
    line = {"x": pose.translation.x, "y": pose.translation.y, "z": pose.translation.z, "timestamp": timestamp}
    with lock:
        pf = pf.append(line, ignore_index=True)
        if pf.shape[0] > 1000:
            pf = pf.iloc[-1000:]


t265 = T265(enable_pose_jumping=False)
t265.register_callback(callback)
t265.start(print_update=False)

try:
    while True:
        # make all axis pair 1:1
        plt.clf()
        with lock:
            if 0:
                plt.subplot(2, 2, 1)
                plt.title("x-y")
                plt.xlabel("x")
                plt.ylabel("y")
                plt.axis("equal")
                plt.plot(0, 0, "ro")
                xy = sns.scatterplot(data=pf, x="x", y="y", hue="timestamp", legend=False, s=10)
                plt.subplot(2, 2, 2)
                plt.title("x-z")
                plt.xlabel("x")
                plt.ylabel("z")
                plt.axis("equal")
                plt.plot(0, 0, "ro")
                xz = sns.scatterplot(data=pf, x="x", y="z", hue="timestamp", legend=False, s=10)
                plt.subplot(2, 2, 3)
                plt.title("y-z")
                plt.xlabel("y")
                plt.ylabel("z")
                plt.axis("equal")
                plt.plot(0, 0, "ro")
                yz = sns.scatterplot(data=pf, x="y", y="z", hue="timestamp", legend=False, s=10)
                plt.subplot(2, 2, 4)
                plt.title("x-y-z")
            else:
                axes = plt.axes(projection="3d")
                axes.scatter3D(pf["z"], pf["x"], pf["y"], c=pf["timestamp"], s=10)
                axes.set_xlabel("z")
                axes.set_ylabel("x")
                axes.set_zlabel("y")
                axes.axis("equal")
        plt.pause(0.05)
finally:
    t265.stop()
