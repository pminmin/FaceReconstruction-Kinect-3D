# FaceReonstruction-Kinect-3D

This is a real-time face reconstruction demo using kinect v2.0. The pipeline is mainly divided into two parts, acquring kinect face data and pcl registration between different frames.

### Dependencies

- PCL
- SFML
- OpenGL

### Usage

First, you need to get `demo.exe` by building the current project, which is used to acquire face data. Then, move to the folder `register` and build the project to get `register.exe`. The final demo is something like `exe/faceModel.py` by the following command:

```
python faceModel.py "×××"
```

For more detailed information, you could refer to my blog.

