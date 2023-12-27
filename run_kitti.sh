cd build
make -j20
cd ..
# ./dsp_slam Vocabulary/ORBvoc.bin configs/KITTI04-12.yaml data/kitti/05 map/05 0
# ./dsp_slam Vocabulary/ORBvoc.bin configs/KITTI04-12.yaml data/half_dataset/07/l map/m 1
./dsp_slam Vocabulary/ORBvoc.bin configs/KITTI04-12.yaml data/half_dataset/07/l map/m 1
# ./dsp_slam Vocabulary/ORBvoc.bin configs/KITTI04-12.yaml data/half_dataset/05/l map/l 0
