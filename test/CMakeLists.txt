# Test inflation
add_executable(test_inflation testInflation.cpp)
target_link_libraries(test_inflation student)

# Test roadmap
add_executable(test_roadmap testRoadMap.cpp)
target_link_libraries(test_roadmap student)

# Test Dijkstra
add_executable(test_dijkstra testDijkstra.cpp)
target_link_libraries(test_dijkstra student)

# Test Collision
add_executable(test_collision testCollision.cpp)
target_link_libraries(test_collision student)

# Test NavMap
add_executable(test_navmap testNavMap.cpp)
target_link_libraries(test_navmap student)

# NavMap benchmark
add_executable(navmap_benchmark benchmarkNavMap.cpp)
target_link_libraries(navmap_benchmark student)