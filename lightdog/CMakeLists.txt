cmake_minimum_required(VERSION 3.8)
project(lightdog)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Python 节点：安装到 lib/<pkg>/，这样 Node(package='lightdog', executable=...) 可用
install(PROGRAMS
  lightdog_led.py
  lightdog_person_detector.py
  lightdog_follower.py
  lightdog_light_auto.py
  lightdog_person_sim.py
  lightdog_twist_mux.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  urdf
  launch
  config
  models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
