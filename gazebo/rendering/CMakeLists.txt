include (${gazebo_cmake_dir}/GazeboUtils.cmake)

include_directories(SYSTEM
  ${OGRE_INCLUDE_DIRS}
  ${CEGUI_INCLUDE_DIRS}
  skyx
  skyx/include
)

if (${OGRE_VERSION} GREATER 1.7.4)
  add_subdirectory(deferred_shading)
endif()
add_subdirectory(skyx)
add_subdirectory(selection_buffer)

set (sources
  ArrowVisual.cc
  AxisVisual.cc
  Camera.cc
  CameraVisual.cc
  COMVisual.cc
  ContactVisual.cc
  Conversions.cc
  DepthCamera.cc
  DynamicLines.cc
  DynamicRenderable.cc
  FPSViewController.cc
  Grid.cc
  GUIOverlay.cc
  Heightmap.cc
  JointVisual.cc
  LaserVisual.cc
  Light.cc
  Material.cc
  MovableText.cc
  OrbitViewController.cc
  Projector.cc
  RenderEngine.cc
  RenderEvents.cc
  Rendering.cc
  Road2d.cc
  RFIDVisual.cc
  RFIDTagVisual.cc
  RTShaderSystem.cc
  Scene.cc
  UserCamera.cc
  VideoVisual.cc
  ViewController.cc
  Visual.cc
  WireBox.cc
  WindowManager.cc
)

set (headers
  ArrowVisual.hh
  AxisVisual.hh
  Camera.hh
  CameraVisual.hh
  COMVisual.hh
  ContactVisual.hh
  Conversions.hh
  DepthCamera.hh
  DynamicLines.hh
  DynamicRenderable.hh
  FPSViewController.hh
  Grid.hh
  GUIOverlay.hh
  Heightmap.hh
  JointVisual.hh
  LaserVisual.hh
  Light.hh
  Material.hh
  MovableText.hh
  OrbitViewController.hh
  Projector.hh
  RenderEngine.hh
  RenderEvents.hh
  Rendering.hh
  RenderTypes.hh
  Road2d.hh
  RFIDVisual.hh
  RFIDTagVisual.hh
  RTShaderSystem.hh
  Scene.hh
  UserCamera.hh
  VideoVisual.hh
  ViewController.hh
  Visual.hh
  WireBox.hh
  WindowManager.hh
  ogre_gazebo.h
  cegui.h
)

set (rendering_headers "" CACHE INTERNAL "rendering headers" FORCE)
foreach (hdr ${headers})
  APPEND_TO_CACHED_STRING(rendering_headers
                          "Rendering Headers" "#include \"rendering/${hdr}\"\n")
endforeach()
configure_file (${CMAKE_CURRENT_SOURCE_DIR}/rendering.hh.in
                ${CMAKE_CURRENT_BINARY_DIR}/rendering.hh)

link_directories(  
  ${ogre_library_dirs}
)

gz_add_library(gazebo_rendering ${sources})

target_link_libraries( gazebo_rendering
  gazebo_common 
  gazebo_sdf_interface
  gazebo_skyx
  gazebo_selection_buffer
  ${ogre_libraries} 
  ${OPENGL_LIBRARIES} 
  ${CEGUI_LIBRARIES}
  X11 
)

if (${OGRE_VERSION} GREATER 1.7.4)
  target_link_libraries(gazebo_rendering gazebo_rendering_deferred)
endif()

gz_install_library(gazebo_rendering)
gz_install_includes("rendering" ${headers} ${CMAKE_CURRENT_BINARY_DIR}/rendering.hh rendering.h)
