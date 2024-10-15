# Set the path to the Firebase SDK libraries
set(FIREBASE_SDK_PATH ${CMAKE_SOURCE_DIR}/Thirdparty/firebase_cpp_sdk)
set(FIREBASE_LIBS_PATH ${FIREBASE_SDK_PATH}/libs/linux/x86_64/cxx11)

# Include directories
include_directories(
    ${FIREBASE_SDK_PATH}/include  # Include the Firebase SDK headers
    ${FIREBASE_LIBS_PATH}
)


# Add the executable

#add_executable(detection detection.cpp)
#add_executable(final final.cpp)
#add_executable(finalApp finalApp.cpp)
add_executable(fireDetection fireDetection.cpp)
#add_executable(SimulatorFireDetector SimulatorFireDetector.cpp)
#add_executable(tryMove tryMove.cpp)



target_link_libraries(fireDetection
    simulator
    ORB_SLAM2
    Auxiliary
    Coordinates
    FirebaseManager
    ${FIREBASE_LIBS_PATH}/libfirebase_database.a
    ${FIREBASE_LIBS_PATH}/libfirebase_auth.a      # Add as needed
    ${FIREBASE_LIBS_PATH}/libfirebase_firestore.a  # Add as needed
    ${FIREBASE_LIBS_PATH}/libfirebase_app.a 
    # Add any other libraries you need
    Threads::Threads
)


#target_link_libraries(finalApp
#    simulator
 #   ORB_SLAM2
  #  Auxiliary
 #   ${FIREBASE_LIBS_PATH}/libfirebase_database.a
  #  ${FIREBASE_LIBS_PATH}/libfirebase_auth.a      # Add as needed
   # ${FIREBASE_LIBS_PATH}/libfirebase_firestore.a  # Add as needed
    #${FIREBASE_LIBS_PATH}/libfirebase_app.a 
    # Add any other libraries you need
    #Threads::Threads
#)


