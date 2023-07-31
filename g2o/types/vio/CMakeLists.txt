add_library(types_vio ${G2O_LIB_TYPE}
  edge_imu.cpp
  edge_imu.h
  edge_imu_bias.cpp
  edge_imu_bias.h
  edge_speed.cpp
  edge_speed.h
  types_vio.cpp
  types_vio.h
  vertex_imu_bias.cpp
  vertex_imu_bias.h
  vertex_speed.cpp
  vertex_speed.h
)


set_target_properties(types_vio PROPERTIES OUTPUT_NAME ${LIB_PREFIX}types_vio)
set_target_properties(types_vio PROPERTIES
  VERSION ${G2O_LIB_VERSION}
  SOVERSION ${G2O_LIB_SOVERSION})
if (APPLE)
  set_target_properties(types_vio PROPERTIES INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
endif()
target_compile_features(types_vio PUBLIC cxx_std_17)
target_link_libraries(types_vio types_slam3d core)
if(G2O_HAVE_OPENGL)
  target_link_libraries(types_vio opengl_helper)
endif()

target_include_directories(types_vio PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include/g2o/types/vio>)

install(TARGETS types_vio
  EXPORT ${G2O_TARGETS_EXPORT_NAME}
  RUNTIME DESTINATION ${RUNTIME_DESTINATION}
  LIBRARY DESTINATION ${LIBRARY_DESTINATION}
  ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
  INCLUDES DESTINATION ${INCLUDES_DESTINATION}
)

file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
install(FILES ${headers} DESTINATION ${INCLUDES_INSTALL_DIR}/types/vio)
