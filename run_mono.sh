cd build
make -j20
cd ..
./dsp_slam_mono Vocabulary/ORBvoc.bin configs/freiburg_001.yaml data/freiburg/001 map/freiburg/001 0

