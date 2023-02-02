# 最小cmake版本
cmake_minimum_required(VERSION 3.8)

# 项目名称
project(AES_LIKE_CIPHERS)

# 设置编译文件的目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin/)

# 添加头文件
include_directories(Lib_Source/Out)
include_directories(Lib_Source)

# 引入math库
link_libraries(m)

# 打包一下c文件列表
aux_source_directory(./Lib_Source lib_s)
aux_source_directory(./Lib_Source/Cipher_Info lib_s_c)
# 编译所有的c文件
add_executable(${PROJECT_NAME} Handle_Source/astbb_main.c ${lib_s} ${lib_s_c})
