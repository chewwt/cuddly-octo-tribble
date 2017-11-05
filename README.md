# cuddly-octo-tribble
## test darkflow for vision

1. set darkflow path in python path

```export PYTHONPATH=$HOME/Repos/darkflow/:$PYTHONPATH```

2. compile cv_bridge from source

```git clone https://github.com/ros-perception/vision_opencv.git
cd /usr/lib/x86_64-linux-gnu/
sudo ln -s libboost_python-py35.so libboost_python3.so
export BOOST_ROOT=/usr/include/boost
```
3. compile
```catkin_make install```
