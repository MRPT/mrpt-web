
file(GLOB SRCS "*.cpp" "*.h")

set(RPC_SPEC_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/mrpt-ws-rpc_spec/include)

set(RAWLOG_SPEC_CLASSNAME CRPCRawLog)
set(RAWLOG_SPEC_OUTPUT ${RPC_SPEC_INCLUDE_DIR}/${RAWLOG_SPEC_CLASSNAME}Abstract.h)
set(RAWLOG_SPEC ${CMAKE_CURRENT_SOURCE_DIR}/${RAWLOG_SPEC_CLASSNAME}.spec.json)

add_executable(mrpt-ws-rpc
	${SRCS}
	${RAWLOG_SPEC_OUTPUT}
)


target_link_libraries(mrpt-ws-rpc
${CMAKE_THREAD_LIBS_INIT}
mrpt-web
libjson-rpc-cpp::server
Boost::system)

include_directories(${RPC_SPEC_INCLUDE_DIR})

add_custom_command(
	OUTPUT ${RAWLOG_SPEC_OUTPUT}
	COMMAND mkdir -p ${RPC_SPEC_INCLUDE_DIR} && ${JSON_RPC_STUB} ${RAWLOG_SPEC} --cpp-server-file=${RAWLOG_SPEC_OUTPUT} --cpp-server=${RAWLOG_SPEC_CLASSNAME}Abstract
)
