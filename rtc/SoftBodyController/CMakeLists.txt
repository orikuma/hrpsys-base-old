set(comp_sources SoftBodyController.cpp ../TorqueFilter/IIRFilter.cpp)
set(libs hrpModel-3.1 hrpUtil-3.1 hrpsysBaseStub)
add_library(SoftBodyController SHARED ${comp_sources})
target_link_libraries(SoftBodyController ${libs})
set_target_properties(SoftBodyController PROPERTIES PREFIX "")

add_executable(SoftBodyControllerComp SoftBodyControllerComp.cpp ${comp_sources})
target_link_libraries(SoftBodyControllerComp ${libs})

set(target SoftBodyController SoftBodyControllerComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)
