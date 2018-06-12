

file(GLOB SRCS "src/*.cpp" "include/mrpt/web/*.h")

add_library(mrpt-web
  SHARED
	${SRCS}
)

target_include_directories(mrpt-web PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(mrpt-web
PUBLIC
${MRPT_LIBS})

install(TARGETS mrpt-web
  RUNTIME DESTINATION lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  PUBLIC_HEADER DESTINATION include
)
