# 进入到build文件夹
if [ ! -d build ]; then
    mkdir build
fi
cd build

# cmake编译
cmake ..
make

# 退出build文件夹
cd ..