from matplotlib import pyplot as plt
def unpackSIFTOctave(kpt):
    """unpackSIFTOctave(kpt)->(octave,layer,scale)
    @created by Silencer at 2018.01.23 11:12:30 CST
    @brief Unpack Sift Keypoint by Silencer
    @param kpt: cv2.KeyPoint (of SIFT)
    """
    _octave = kpt.octave
    octave = _octave & 0xFF
    layer = (_octave >> 8) & 0xFF
    if octave >= 128:
        octave |= -128
    if octave >= 0:
        scale = float(1/(1 << octave))
    else:
        scale = float(1 << -octave)
    return (octave, layer, scale)


def get_kps_scales(kps):
    scale = [unpackSIFTOctave(kp)[0] for kp in kps]
    return scale
def get_kps_size(kps):
    sizes = [kp.size for kp in kps]
    return sizes

def draw_pyramid_distribution(kps):
    from collections import Counter
    levels = get_kps_scales(kps)
    fig = plt.figure(figsize=(5,5))
    ax  = plt.gca()
    arr =ax.hist(levels)
    for i in range(len(arr[0])):
        plt.text(arr[1][i],arr[0][i],str(arr[0][i]))
    plt.show()


def splitFeatureBySize(feature, descriptor):
    from collections import defaultdict
    scale = [kp.size  for kp in feature]
    
    index = defaultdict(list)
    for i, s in enumerate(scale):
        index[s].append(i)
    res = defaultdict(dict)
    for k, ind in index.items():
        res[k] = {
            "feature": [feature[i] for i in ind],
            "descriptor":[descriptor[i] for i in ind]
        }
    return res

def splitFeatureByScale(feature, descriptor):
    from collections import defaultdict
    scale = [unpackSIFTOctave(kp)[2]  for kp in feature]
    index = defaultdict(list)
    for i, s in enumerate(scale):
        index[s].append(i)
    res = defaultdict(dict)
    for k, ind in index.items():
        res[k] = {
            "feature": [feature[i] for i in ind],
            "descriptor":[descriptor[i] for i in ind]
        }
    return res