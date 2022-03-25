import numpy as np
import cv2


def extract(img, num, level=8, method="ORB"):
    if method == "SIFT":
        if cv2.__version__[0] == '3':
            sift = cv2.xfeatures2d.SIFT_create(num, nOctaveLayers=level)
        else:
            sift = cv2.SIFT_create(num, nOctaveLayers=level)
        kps, deps = sift.detectAndCompute(img, None)
        return kps, deps
    elif method == "ORB":
        orb = cv2.ORB_create(num, nlevels=level)
        kps, deps = orb.detectAndCompute(img, None)
    return kps, deps


def getmatcher(method="ORB"):
    matcher = cv2.BFMatcher() if method != "ORB" else cv2.BFMatcher(cv2.NORM_HAMMING2)
    return matcher


def match(deps1,  deps2, matcher, good_threshold=0.6):
    matches = matcher.knnMatch(deps1, deps2, k=2)
    good = []
    for m, n in matches:
        if m.distance < good_threshold*n.distance:
            good.append(m)
    return good
