find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

include_directories(
	${EIGEN3_INCLUDE_DIR}
    ${BULLET_DIR}/src
    ${OpenRAVE_INCLUDE_DIRS}
    ${BULLETSIM_SOURCE_DIR}/src
    ${OSGBULLET_DIR}
	${WM_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS} ${BULLETSIM_BINARY_DIR}/src ${WM_LIBRARY_DIR} ${WM_ROOT})

add_executable(test_simplescene test_simplescene.cpp)
target_link_libraries(test_simplescene simulation robots)

add_executable(test_softbody test_softbody.cpp)
target_link_libraries(test_softbody simulation)

add_executable(test_cloth_grasping test_cloth_grasping.cpp)
if(VTK_LIBRARIES)
	target_link_libraries(test_cloth_grasping simulation robots clouds ${VTK_LIBRARIES} ${WM_LIBRARIES} ${PCL_LIBRARIES})
else()
	target_link_libraries(test_cloth_grasping simulation robots clouds vtkHybrid ${WM_LIBRARIES} ${PCL_LIBRARIES})
endif()

add_executable(test_rigidbody_grasping test_rigidbody_grasping.cpp)
target_link_libraries(test_rigidbody_grasping simulation robots)

#add_executable(test_playback_joints test_playback_joints.cpp)
#target_link_libraries(test_playback_joints simulation)

add_executable(test_rope test_rope.cpp)
target_link_libraries(test_rope simulation robots)

add_executable(test_fork test_fork.cpp)
target_link_libraries(test_fork simulation)

add_executable(test_fork_softbody test_fork_softbody.cpp)
target_link_libraries(test_fork_softbody simulation)

add_executable(test_shirt test_shirt.cpp)
target_link_libraries(test_shirt simulation)

add_executable(test_collision_detection test_collision_detection.cpp)
target_link_libraries(test_collision_detection simulation)

add_executable(test_action test_action.cpp)
target_link_libraries(test_action simulation)

add_executable(test_scene_record test_scene_record.cpp)
target_link_libraries(test_scene_record simulation)
