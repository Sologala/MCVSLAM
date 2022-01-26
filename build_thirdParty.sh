cd ThirdParty

mkdir installed
cd g2o
mkdir build
cd build

cmake .. -DCMAKE_INSTALL_PREFIX="../../installed" -DCMAKE_BUILD_TYPE=Release
make -j
make install 
cd ../../

cd cv_bridge
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX="../../installed" -DCMAKE_BUILD_TYPE=Release
make -j
make install
cd ../../

cd ../