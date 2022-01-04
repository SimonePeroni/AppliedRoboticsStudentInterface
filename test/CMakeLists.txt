add_executable(test_inflation main.cpp
               test_dubins testDub.cpp)
target_link_libraries(test_inflation student
                      test_dubins student)
