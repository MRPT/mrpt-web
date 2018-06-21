file(GLOB SRCS "*.cpp" "*.h")

add_executable(mrpt-ws-rpc
	${SRCS}
)

target_link_libraries(mrpt-ws-rpc
PUBLIC
${CMAKE_THREAD_LIBS_INIT}
mrpt-web
jsoncpp_lib_static
Boost::system
OpenSSL::SSL
OpenSSL::Crypto
${MRPT_LIBS})
