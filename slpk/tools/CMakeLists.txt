define_module(BINARY slpk2obj
  DEPENDS slpk service
)

set(slpk2obj_SOURCES slpk2obj.cpp)
add_executable(slpk2obj ${slpk2obj_SOURCES})
target_link_libraries(slpk2obj ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(slpk2obj PRIVATE ${MODULE_DEFINITIONS})
buildsys_binary(slpk2obj)
