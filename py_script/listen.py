from matplotlib.backends.backend_qt5 import NavigationToolbar2QT
import os

from sklearn.preprocessing import scale
from rospy.core import is_shutdown
from threading import Lock, Thread
import threading
from queue import Queue
from matplotlib import pyplot as plt
import matplotlib
import copy
import rospy
import argparse
import numpy as np
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String
tracj_gt = None
state_lock = Lock()
is_done = False


class Listener:
    def __init__(self, axs, topic_type,  topic_name, draw_callback) -> None:
        self.data_queue = Queue(1)
        self.topic_name = topic_name

        self.sub_callback = (lambda data, q=self.data_queue:  q.put(
            data) if not q.full() else None)
        self.draw_callback = draw_callback
        self.sub = rospy.Subscriber(
            topic_name, topic_type, callback=self.sub_callback)
        self.axs = axs

    def process(self):
        if (not self.data_queue.empty()):
            data = self.data_queue.get()
            self.draw_callback(self.axs, data)

def calError(axs, tracj, gt, method = "RPE"):
    if (len(tracj)  < 2): return
    Errors = []
    if method == "APE":
        for p1, p2 in zip(tracj, gt):
            err = np.abs(p1 - p2)
            Errors.append(err)
    elif method == "RPE":
        _interval = 10
        for i in range(1, len(tracj)):
            t0, t1 = tracj[i- 1], tracj[i]
            g0, g1 = gt[i- 1], gt[i]
            err = np.abs((t1 - t0) - (g1 - g0))
            # print(t1 - t0, g1 - g0, err)
            Errors.append(err)
    else:
        print("Non-support error method")
        return None

    Errors = np.array(Errors)
    # drew xyz
    errs = np.linalg.norm(Errors, axis=1)
    plt.autoscale(enable=True, axis='y')
    axs[0].cla()
    axs[0].plot(errs)
    axs[0].set_title("xyz err")
    means = np.mean(errs)
    ylim_ = axs[0].get_ylim()[-1]
    axs[0].text(0, ylim_ * 0.9, "{:.6f}".format(means))
    axs[0].axhline(y=means, color="red")

    # drew xy
    errs = np.linalg.norm(Errors[:, :1], axis=1)
    axs[1].cla()
    axs[1].plot(errs)
    axs[1].set_title("xy plane err")
    means = np.mean(errs)
    ylim_ = axs[1].get_ylim()[-1]
    axs[1].text(0, ylim_ * 0.9, "{:.6f}".format(means))
    axs[1].axhline(y=means, color="red")

    # drew z
    errs = Errors[:, 2]
    axs[2].cla()
    axs[2].plot(errs)
    axs[2].set_title("z err")
    means = np.mean(errs)
    ylim_ = axs[2].get_ylim()[-1]
    axs[2].text(0, ylim_ * 0.9, "{:.6f}".format(means))
    axs[2].axhline(y=means, color="red")


def pose_draw_callback(axs, data):
    global tracj_gt
    tracj = []
    for pose in data.poses:
        q = [pose.orientation.w,  pose.orientation.x,
             pose.orientation.y, pose.orientation.z]
        t = [pose.position.x, pose.position.y, pose.position.z]
        tracj.append(t)
    gt = tracj_gt[: len(tracj)].copy()
    tracj, gt = np.array(tracj), np.array(gt)

    # 修正 tracj的 x 坐标
    tracj[:,0] *= -1
    calError(axs, tracj, gt, method = "RPE")


def time_cost_draw_callback(axs, data):
    # data is std_msgs/String
    str_data = data.data.split("\n")
    str_data = [l.strip().split('|')for l in str_data if len(l.strip())]
    dict_data = {
        l[0]: list(map(float, l[1:]))
        for l in str_data
    }
    times = [times[0] for name, times in dict_data.items()]
    labels = [name for name, times in dict_data.items()]
    # print(labels)
    # print(times)
    axs[0].cla()
    axs[0].bar(range(len(times)), times, tick_label=labels, fc='g')

    for x, y in enumerate(times):
        plt.text(x+0.05, y+0.05, '{:.2f}'.format(y), ha='center', va='bottom')

    # axs[0].set_xs(rotation=300)
    # print(items)


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
    rate = rospy.Rate(100)  # 10hz
    while not rospy.is_shutdown():
        state_lock.acquire()
        if is_done == True:
            print("close ros caused by wnd")
            break
        state_lock.release()
        rate.sleep()
    print("close on ros")
    state_lock.acquire()
    is_done = True
    state_lock.release()


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

    rospy.init_node('talker', anonymous=True)
    ros_thread = Thread(target=ros_loop)

    plt.tight_layout()
    plt.ion()
    fig = plt.figure(figsize=(10, 10))
    fig.show()

    #    create lister
    listen_pose = Listener(
        axs=[plt.subplot(4, 1, 1), plt.subplot(4, 1, 2), plt.subplot(4, 1, 3)],
        topic_type=PoseArray, topic_name="MCVSLAM/tracj",
        draw_callback=pose_draw_callback
    )

    listen_time = Listener(
        axs=[plt.subplot(4, 1, 4)],
        topic_type=String, topic_name="MCVSLAM/Time",
        draw_callback=time_cost_draw_callback
    )

    ros_thread.start()

    draw_cnt = 0
    while True:
        listen_pose.process()
        listen_time.process()

        state_lock.acquire()
        if is_done == True:
            print("close caused by ros")
            break
        state_lock.release()

        fig.canvas.draw_idle()
        fig.canvas.start_event_loop(0.1)
