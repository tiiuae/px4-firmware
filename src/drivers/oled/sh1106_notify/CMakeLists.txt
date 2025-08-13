px4_add_module(
  MODULE drivers__oled__sh1106_notify
  MAIN sh1106_notify
  COMPILE_FLAGS
    -DDEBUG_BUILD
  SRCS
    sh1106_notify.cpp
    sh1106.cpp
  HDRS
    sh1106.hpp
    font6x8.hpp
  DEPENDS
    modules__uORB
    px4_work_queue
)
