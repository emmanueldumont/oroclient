cmake_minimum_required(VERSION 2.8.3)


add_executable(oroclient oroclient.cpp)
# Oroclient needs sources from the "src" folder:
add_subdirectory(lib)
target_link_libraries(oroclient messageParser manageCommand utils ${catkin_LIBRARIES})
add_dependencies(oroclient oroclient_generate_messages_cpp)
