cmake_minimum_required(VERSION 3.8)
project(ouster_slam)

# Kommentare auf Deutsch
# Dieses Paket installiert nur Launch- und (optional) Param-Dateien.

find_package(ament_cmake REQUIRED)

# launch/ ins Install-Ziel kopieren
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# (Optional) params/ mit YAMLs mitinstallieren
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/params")
  install(DIRECTORY params
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()

