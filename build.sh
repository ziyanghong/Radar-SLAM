echo "Configuring and building M2DP ..."

cd Thirdparty/codegen/lib
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

