cd ThirdParty

mkdir installed
cd g2o
mkdir build
cd build

cmake .. -DCMAKE_INSTALL_PREFIX="../../installed"
make -j
make install 
cd ../../

