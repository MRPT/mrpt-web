set(LIB_SRC
  CRawlogTreeProcessor.cpp
)

add_library(lib_rawlog_app ${LIB_SRC})

add_executable(rawlog-viewer
	main.cpp
)
target_link_libraries(lib_rawlog_app
PUBLIC
jsoncpp_lib_static
mrpt-web
)
target_link_libraries(rawlog-viewer
PUBLIC
lib_rawlog_app
${CMAKE_THREAD_LIBS_INIT}
mrpt-web
jsoncpp_lib_static
Boost::system
OpenSSL::SSL
OpenSSL::Crypto
${MRPT_LIBS})
