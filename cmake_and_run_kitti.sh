cd build
cmake \
  -DOpenCV_DIR="$(pwd)/../Thirdparty/opencv/build" \
  -DEigen3_DIR="$(pwd)/../Thirdparty/eigen/install/share/eigen3/cmake" \
  -DPangolin_DIR="$(pwd)/../Thirdparty/Pangolin/build" \
  -DPYTHON_LIBRARIES="$conda_env_dir/lib/libpython3.7m.so" \
  -DPYTHON_INCLUDE_DIRS="$conda_env_dir/include/python3.7m" \
  -DPYTHON_EXECUTABLE="$conda_env_dir/bin/python3.7" \
  ..
  
make -j20
cd ..
./dsp_slam Vocabulary/ORBvoc.bin configs/KITTI04-12.yaml data/kitti/07 map/kitti/07 1
