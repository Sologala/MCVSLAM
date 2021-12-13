# Multi Camera Visual SLAM 
This repo is aim to realise a simple visual slam system which support multi camera configruation. Meanwhile, we also utilize the OpensceneGraph to simulate some drone motion scene with groundtrugh trajectory, also use it to visulize our sparse mapping result, and try to find some strategies to improve the system.

Currently, I am mainly use it to build a convinent platform for accomplish my thesis. So I have no more energy to manage pull request and merge request, Any idea or question or suggestion is welcome to be descuss in issues.

# Feature
- Multi camera hierarchical optimization based on multi resolution cameras observation.
- A multi thread Framework similar to ORB_SLAM but more simple and readable.
- Unified Matching process code  as well as  Optimizing  to reduce redundancy


# Dependency 
- [fmt](https://github.com/fmtlib/fmt) **For Log and formating console output**
- [cmdline](https://github.com/tanakh/cmdline)
- [yaml](https://github.com/jimmiebergmann/mini-yaml) **Base on it , and support parsing array type.**
- [ROS melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) 
- [opencv](https://github.com/opencv/opencv) 4+  with  [contrib](https://github.com/opencv/opencv_contrib)
- [Open Scene Graph](https://github.com/openscenegraph/OpenSceneGraph)



# TODO 
- [X] [Capture](./modules/capture/README.md)
- [ ] Image Rectification && pre-process (`currently, test data are simulated from osg, no need to do`)  
- [X] local feature extractor (ORB | SURF | Super Point) \ matcher
- [X] [OSG](https://github.com/openscenegraph/OpenSceneGraph) Viewer 、Visulization Tracjtory and Camera and Maps.
- [X] Visual Odemetry
- [X] Local Mapping with Essential Graph
- [ ] LoopClosing with Dbow 
- [X] Visualization of Essentialgraph
- [X] [osg_viewer interaction specification](modules/osg_viewer/README.md)
- [ ] Quantification of Reprojection Error with Scene Model,  Calculated by 
- [ ] Build instruction (`If someone is intrested on this prototype`)
- [ ] Quantification of ATE or RTE.

    $$ e = ||z_{messured} - z_{buffer_from osg} ||_2^2$$

# Multi Camera Configuration


```
                ┌─────────  stereo────────┐
                │                         │
                │                         │
                │                         │
                │                         │
                ▼           110           ▼
                30     ┌──────────┐      30
             ┌──────┐  │          │   ┌──────┐
             │      │  │          │   │      │
 body center │ left │  │    wide  │   │ right│
             └──x───┘  │          │   └───x──┘
                x      └─────x────┘       x
                x            x            x
                x            x            x
                x            x            x
                x            x            x
                x            x            x
                ───────────────────────────────────►
                0            0.5m         1.0m
```



# Time Performance

2000 ORB feature points in each image, running on i7-9700 with single thread.

```shell
        item              time(ms)              fps         
      Local BA           169.727319           5.891800      
 Track MotionModel        6.184962           161.679858     
  Track Local Map        26.733954           37.405476      
    ORB_EXTRACT           6.437734           155.331754     
  Stereo Matching        23.619861           42.337073      
Frame Triangulation      34.983119           28.585134      
Local kf UpdateConnection      5.755037           173.757820     
    Local Fuse           22.248238           44.947178      
 Track LastKeyFrame      16.244335           61.559545  
```





![](./.readme/demo.gif)

![](./.readme/align_result.png)
![](./.readme/osg_model_visulization.png)


