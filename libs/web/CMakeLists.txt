

file(GLOB SRCS "src/*.cpp" "include/mrpt/webs/*.h")

add_library(mrpt-web
  SHARED
	${SRCS}
)

target_include_directories(mrpt-web PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

install(TARGETS mrpt-web
  RUNTIME DESTINATION lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  PUBLIC_HEADER DESTINATION include
)
