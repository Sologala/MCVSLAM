from matplotlib.pyplot import ylim
import numpy as np
import cv2
from pyfbow import Vocabulary
from sklearn.preprocessing import scale

class ScaleMatcher:
    def __init__(self, voc_path: str) -> None:
        self.voc = Vocabulary(verbose = True)
        self.voc.readFromFile(voc_path)
                


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('-v', "--voc", default="./sift_visdrone.voc")
    args = parser.parse_args()
    mm = ScaleMatcher(args.voc)
    
    



    # img = cv2.imread("../data/left/1639379415303512368.png",cv2.IMREAD_GRAYSCALE)
    # img = cv2.imread("../data/left/1639379415320128733.png",cv2.IMREAD_GRAYSCALE)
    img = cv2.imread("../data/left/1639379415336847758.png",cv2.IMREAD_GRAYSCALE)
    




    import FeatureExtract as F
    npoint = 2000
    method = "SIFT"
    matcher = F.getmatcher(method)

    kps, desps = F.extract(img, npoint, 3, method)
    bowfeature = mm.voc.transform_with_feature(desps, 3)
    print(max(bowfeature.keys()), min(bowfeature.keys()))
    print(type(bowfeature))
    histdata = []
    for k , v in bowfeature.items():
        print(k, v)
        for i in range(len(v)):
            histdata.append(k)

    from matplotlib import pyplot as plt
    arrs = plt.hist(histdata, len(histdata), (0, max(histdata)),density= True)
    # print(arrs)
    plt.savefig("1.png")
    plt.show()
    


