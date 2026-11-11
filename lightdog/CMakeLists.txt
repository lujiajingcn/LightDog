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
  lightdog_aruco_dock.py
  lightdog_dock_manager.py
  lightdog_recorder.py
  lightdog_server.py
  lightdog_fall_detector.py
  lightdog_alert_manager.py
  lightdog_mission.py
  lightdog_voice.py
  lightdog_mcu_bridge.py
  lightdog_raspbot_driver.py
  DESTINATION lib/${PROJECT_NAME}
)

# 共享的最简模块（不是可执行节点）。装在同一个目录下，各节点用
# `from lightdog_qos import SAFETY_QOS` 就能拿到 —— 脚本自身的目录本来
# 就在 sys.path[0]，不需要额外的 package 结构。
install(FILES
  lightdog_qos.py
  lightdog_call.py
  lightdog_mcu_proto.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  urdf
  launch
  config
  models
  web
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
