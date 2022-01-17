from geometry_msgs.msg import PoseArray
import numpy as np
import argparse
import rospy
import copy
import matplotlib
from matplotlib import pyplot as plt
from queue import Queue
import threading
from threading import Lock, Thread
from rospy.core import is_shutdown
import os
from matplotlib.backends.backend_qt5 import NavigationToolbar2QT


tracj_gt = []

axs = None
titles = ["xyz", "xy", "z"]
matrics = ["m", "m", "m"]

# matplotlib.use('TkAgg')
matplotlib.use("Qt5agg")
fig = None

data_queue = Queue(1)
is_done = False
state_lock = Lock()


def APE(tracj, gt):
    global axs, fig

    Errors = []
    for p1, p2 in zip(tracj, gt):
        p1[0] *= -1
        # p1[2] *= -1
        # print(p1, p2)
        err = np.abs(p1 - p2)
        Errors.append(err)

    Errors = np.array(Errors)

    # for i in range(1, len(titles)):
    #     axs[i - 1].plot(Errors[:, i - 1])
    #     axs[i-1].set_title(titles[i-1])
    #     means = np.mean(Errors[:, i - 1])
    #     axs[i-1].axhline(y=means, color="red")
    #     axs[i-1].set_ylim((0, 1.5))

    # drew xyz
    errs = np.linalg.norm(Errors, axis=1)
    axs[0].plot(errs)
    axs[0].set_title(titles[0])
    means = np.mean(errs)
    axs[0].text(0, 1.2, "{:.6f}".format(means))
    axs[0].axhline(y=means, color="red")
    axs[0].set_ylim((0, 1.5))

    # drew xy
    errs = np.linalg.norm(Errors[:, :1], axis=1)
    axs[1].plot(errs)
    axs[1].set_title(titles[1])
    means = np.mean(errs)
    axs[1].text(0, 1.2, "{:.6f}".format(means))
    axs[1].axhline(y=means, color="red")
    axs[1].set_ylim((0, 1.5))

    # drew z
    errs = Errors[:, 2]
    axs[2].plot(errs)
    axs[2].set_title(titles[2])
    means = np.mean(errs)
    axs[2].text(0, 1.2, "{:.6f}".format(means))
    axs[2].axhline(y=means, color="red")
    axs[2].set_ylim((0, 1.5))

# fig.canvas.draw_idle()
# fig.canvas.flush_events()


def pose_sub_callback(data):
    # print(data.poses.position)

    tracj = []
    for pose in data.poses:
        q = [pose.orientation.w,  pose.orientation.x,
             pose.orientation.y, pose.orientation.z]
        t = [pose.position.x, pose.position.y, pose.position.z]
        tracj.append(t)
    if data_queue.full():
        return
    else:
        data_queue.put(tracj)


def ros_msg2tracj(data):
    tracj = []


def load_groundtruth(_gt_file):
    tracj = []
    with open(_gt_file, "r") as f:
        lines = f.readlines()
        data = [list(map(float, line.strip().split(' ')))
                for line in lines]
        data = np.array(data)[:, 1:]
        # [x, y, z, w, dx, dy, dz]
        for d in data:
            q = d[3:]
            t = d[:3]
            # rot = lie.quanternion2so3(q)
            tracj.append(t)
    return tracj


def ros_loop():
    global is_done, state_lock
    rate = rospy.Rate(30)  # 10hz
    while not rospy.is_shutdown():
        rate.sleep()
    print("done")
    state_lock.acquire()
    is_done = True
    state_lock.release()


def mypause(interval):
    backend = plt.rcParams['backend']
    if backend in matplotlib.rcsetup.interactive_bk:
        figManager = matplotlib._pylab_helpers.Gcf.get_active()
        if figManager is not None:
            canvas = figManager.canvas
            if canvas.figure.stale:
                canvas.draw()
            canvas.start_event_loop(interval)
            return

# wnd_button_event_callback


save_figure = NavigationToolbar2QT.save_figure


def on_save(self, *args, **kwargs):
    print('save_event')

    plt.savefig("/home/wen/aa_pic/fig.png",  bbox_inches="tight")
    os.system("scp /home/wen/aa_pic/fig.png wen@mac:~/Desktop/screenShot")
    # save_figure(self, *args, **kwargs)


NavigationToolbar2QT.save_figure = on_save

if __name__ == "__main__":

    # argument parsing
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-g", "--gt", help="the ground truth file to evaluation")
    args = parser.parse_args()
    print("Ground Truth File is ", args.gt)
    tracj_gt = load_groundtruth(args.gt)
    ros_thread = Thread(target=ros_loop)
    rospy.init_node('talker', anonymous=True)
    sub = rospy.Subscriber("MCVSLAM/tracj", PoseArray,
                           callback=pose_sub_callback)
    ros_thread.start()

    if axs == None:
        plt.tight_layout()
        plt.ion()
        fig = plt.figure(figsize=(10, 10))
        # fig.canvas.mpl_connect("button_press_event", on_press)
        axs = [plt.subplot(3, 1, 1), plt.subplot(
            3, 1, 2),    plt.subplot(3, 1, 3)]
        fig.show()

    draw_cnt = 0
    while True:
        if (not data_queue.empty()):
            for i in range(1, 4):
                axs[i-1].cla()
            tracj = data_queue.get()
            gt = tracj_gt[: len(tracj)].copy()
            APE(tracj, gt)
            draw_cnt += 1
        state_lock.acquire()
        if is_done == True:
            print("down!!!!")
            break
        state_lock.release()

        # mypause(0.005)

        fig.canvas.draw_idle()
        fig.canvas.start_event_loop(0.001)
