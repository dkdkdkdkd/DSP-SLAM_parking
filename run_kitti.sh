cd build
make -j20
cd ..
./dsp_slam Vocabulary/ORBvoc.bin configs/KITTI04-12.yaml data/kitti/07 map/07 0
