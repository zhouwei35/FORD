find_package(yaml-cpp REQUIRED QUIET)

include_directories(
	/usr/local/include  # 添加yaml-cpp头文件路径
)

list(APPEND ALL_TARGET_LIBRARIES /usr/local/lib/libyaml-cpp.so)  # 添加yaml-cpp库文件路径