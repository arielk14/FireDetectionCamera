cmake_minimum_required(VERSION 3.0)
project(fireDetection)
set(CMAKE_CXX_STANDARD 17)
IF (NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Debug)
ENDIF ()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})
IF (WIN32)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   -MD -bigobj")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -MD -bigobj")
ELSE (WIN32)
    set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -O0 -fPIC" )
    set( CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -Wall -O0 -fPIC" )
    if(CMAKE_BUILD_TYPE STREQUAL "RELEASE")
        set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native" )
        set( CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -Wall -O3 -march=native" )
    endif ()
ENDIF ()

add_definitions(-DEIGEN_NO_DEBUG)
add_definitions(-DEIGEN_NO_STATIC_ASSERT)
add_definitions(-DEIGEN_FAST_MATH)
add_definitions(-DEIGEN_VECTORIZE)
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
list(APPEND VCPKG_FEATURE_FLAGS "versions")
find_package(nlohmann_json REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS system serialization)
find_package(OpenCV REQUIRED)



include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${PYTHON_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/
        ${PROJECT_SOURCE_DIR}/Thirdparty
        ${PROJECT_SOURCE_DIR}/Thirdparty/slam/include/
       
)





add_subdirectory(Thirdparty)

# Does not inherit from subdirectory, hence need to rerun
find_package(Pangolin REQUIRED PATHS "${CMAKE_CURRENT_SOURCE_DIR}/Pangolin")

include_directories(
        ${Pangolin_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/utils
        ${PROJECT_SOURCE_DIR}/tools
        ${PROJECT_SOURCE_DIR}/tools/simulator/
        ${PROJECT_SOURCE_DIR}/include/
        ${PROJECT_SOURCE_DIR}/libs/
        ${PROJECT_SOURCE_DIR}/Thirdparty/firebase_cpp_sdk/include
)

add_library(Auxiliary STATIC
        src/Auxiliary.cpp
)

# Add libraries for Coordinates and FirebaseManager
add_library(Coordinates STATIC src/Coordinates.cpp)
add_library(FirebaseManager STATIC src/FirebaseManager.cpp)


target_link_libraries(Auxiliary PUBLIC
        ${Pangolin_LIBRARIES}
)

add_library(simulator src/simulator/simulator.cpp)

# Linking ORB_SLAM2, Auxiliary, and Firebase
target_link_libraries(simulator
    ORB_SLAM2
    Auxiliary
)
set(FIREBASE_SDK_PATH ${PROJECT_SOURCE_DIR}/Thirdparty/firebase_cpp_sdk/libs/linux/x86_64/cxx11)
# Add Firebase libraries to the simulator target
target_link_libraries(simulator
    ${FIREBASE_SDK_PATH}/libfirebase_app.a
    ${FIREBASE_SDK_PATH}/libfirebase_database.a
    ${FIREBASE_SDK_PATH}/libfirebase_auth.a      # Add any additional Firebase libraries you need
    ${FIREBASE_SDK_PATH}/libfirebase_firestore.a  # Add as needed
)


add_subdirectory(FireDetectionApp)


