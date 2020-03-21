# ShitVO

## Dependency

- **Eigen**

  ```bash
  sudo apt-get install libeigen3-dev
  ```

- **OpenCV**

  I prefer to compile opencv from [source](https://github.com/opencv/opencv) with some extra modules like `aruco`, `viz`, and so on. If you encounter some extra module errors due to kinds of reasons, just Skip them! For example,

  ```bash
  cmake .. -D BUILD_opencv_ts=OFF -D BUILD_opencv_text=OFF
  make -j8
  ```

- **OpenCV_viz(Optional)**

  If you want to see the real-time camera trajectory, you shall install viz module.

  1. Compile `VTK` from [source](https://github.com/Kitware/VTK). Usually, v7.x.x is enough.

  2. Recompile OpenCV with [contrib](https://github.com/opencv/opencv_contrib). Check that version must match to `OpenCV` above.

     ```bash
     cmake .. –D WITH_VTK=ON -D VTK_DIR="/your/vtk/build/"
     ```



## Run demo

```bash
mkdir build
cd build
cmake ..
make -j8
./bin/demo_homo_vo ../config/default.yaml /your/dataset/path
```

BTW, you can specify opencv_dir like

    ```bash
cmake .. -D OpenCV_DIR="your/opencv/build"
    ```

The dataset directory is arranged as following:

```
cam
  |_1234567890.jpg
  |_1234567891.jpg
  |_1234567892.jpg
  |_...
img_list.txt
imu_data.txt
```

- img_list.txt

  Contains all images file names.

- imu_data.txt

  Data recorded as following:

  ```
  timestamp(unit: ms),acc[3],gry[3],att[9]
  ```

  



