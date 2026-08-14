find_package(camera_interfaces REQUIRED)

install(PROGRAMS
  camera_module/camera_node.py
  DESTINATION lib/${PROJECT_NAME}
)
