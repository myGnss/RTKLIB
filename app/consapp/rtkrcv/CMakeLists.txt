include_directories(${PROJECT_SOURCE_DIR}/src)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

set(CMAKE_CXX_STANDARD 20)
add_executable(rtkrcv rtkrcv.cpp)
target_link_libraries(rtkrcv rtklib)
install(TARGETS rtkrcv RUNTIME DESTINATION bin)
