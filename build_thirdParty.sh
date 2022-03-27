rebuild=$1

cd ThirdParty
if [ ! -d "installed" ]; then
        mkdir installed
fi

cd g2o

if [ $rebuild="1" ]; then
        echo "rebuild"
        rm -rf build
fi

mkdir build
cd build

cmake .. -DCMAKE_INSTALL_PREFIX="../../installed" \
        -DCMAKE_BUILD_TYPE=Release\
        -DG2O_BUILD_EXAMPLES=0
make -j
make install 
cd ../../

cd cv_bridge
if [ $rebuild="1" ]; then
        echo "rebuild"
        rm -rf build
fi
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX="../../installed" -DCMAKE_BUILD_TYPE=Release
make -j
make install
cd ../../

cd ../
