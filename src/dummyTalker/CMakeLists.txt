cmake_minimum_required(VERSION 2.8.3)


add_executable(dummyTalker dummyTalker.cpp)
target_link_libraries(dummyTalker ${catkin_LIBRARIES})
add_dependencies(dummyTalker oroclient_generate_messages_cpp)
