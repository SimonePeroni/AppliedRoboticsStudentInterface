# Test inflation
add_executable(test_inflation testInflation.cpp)
target_link_libraries(test_inflation student)

# Test dubins
#add_executable(test_dubins testDub.cpp)
#target_link_libraries(test_dubins student)

# Test roadmap
add_executable(test_roadmap testRoadMap.cpp)
target_link_libraries(test_roadmap student)