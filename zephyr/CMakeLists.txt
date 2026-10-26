# IEEE 802.15.4 OTNS out-of-source driver

if(CONFIG_IEEE802154_OTNS)

  set(otns_drv_dir ${CMAKE_CURRENT_SOURCE_DIR}/drivers/ieee802154)

  zephyr_library()
  zephyr_library_sources(${otns_drv_dir}/ieee802154.c)
  zephyr_library_sources(${otns_drv_dir}/ieee802154_frame.c)
  zephyr_library_sources(${otns_drv_dir}/ieee802154_frame_parse.c)
  zephyr_library_sources(${otns_drv_dir}/ieee802154_isr.c)
  zephyr_library_sources(${otns_drv_dir}/alarm_isr.c)
  zephyr_library_sources(${otns_drv_dir}/cli.c)
  zephyr_library_sources(${otns_drv_dir}/host_msg.c)
  zephyr_library_sources(${otns_drv_dir}/radio_cca.c)

  get_filename_component(otns_dir ${CMAKE_CURRENT_SOURCE_DIR}/.. ABSOLUTE)
  set(ot_dir ${otns_dir}/openthread)
  zephyr_library_sources(${ot_dir}/examples/platforms/utils/mac_frame.cpp)
  set_source_files_properties(${ot_dir}/examples/platforms/utils/mac_frame.cpp
    PROPERTIES INCLUDE_DIRECTORIES "${ot_dir}/src;${ot_dir}/examples/platforms/utils")
  zephyr_library_include_directories(${ot_dir}/src/core)
  target_link_libraries(${ZEPHYR_CURRENT_LIBRARY} PRIVATE ot-config)
  zephyr_library_sources_ifdef(CONFIG_OPENTHREAD_BORDER_ROUTING ${otns_dir}/ot-rfsim/src/infra_if.c)
  if(CONFIG_OPENTHREAD_BORDER_ROUTING AND CONFIG_OPENTHREAD_UDP_FORWARD)
    zephyr_library_sources(${otns_drv_dir}/host_forward.c)
  endif()

  zephyr_ld_options(
    -Wl,--wrap=otPlatRadioGetCcaEnergyDetectThreshold
    -Wl,--wrap=otPlatRadioSetCcaEnergyDetectThreshold
    -Wl,--wrap=otPlatAlarmMilliGetNow
    -Wl,--wrap=otPlatAlarmMilliStartAt
    -Wl,--wrap=otPlatAlarmMilliStop
    -Wl,--wrap=otPlatAlarmMicroGetNow
    -Wl,--wrap=otPlatAlarmMicroStartAt
    -Wl,--wrap=otPlatAlarmMicroStop
    -Wl,--wrap=platformAlarmMilliInit
    -Wl,--wrap=platformAlarmMicroInit
    -Wl,--wrap=platformAlarmMilliProcess
    -Wl,--wrap=platformAlarmMicroProcess
    -Wl,--wrap=alarm_milli_set_time_offset_ms
  )
  if(CONFIG_OPENTHREAD_BORDER_ROUTING AND CONFIG_OPENTHREAD_UDP_FORWARD)
    zephyr_ld_options(-Wl,--wrap=openthread_set_receive_cb)
  endif()

  zephyr_include_directories(${otns_drv_dir})
  zephyr_include_directories(${otns_dir})
  zephyr_include_directories(${otns_dir}/ot-rfsim/src)

  target_compile_options(native_simulator BEFORE INTERFACE
    -I${otns_drv_dir}
    -I${otns_dir}
    -I${otns_dir}/openthread/include
    -I${otns_dir}/openthread/src/core
    -I${otns_dir}/openthread/src
    -I${otns_dir}/ot-rfsim/src
    -DOPENTHREAD_PROJECT_CORE_CONFIG_FILE=\\\"openthread-core-rfsim-config.h\\\"
  )
  target_sources(native_simulator INTERFACE ${otns_drv_dir}/ieee802154_bottom.c)
  target_sources(native_simulator INTERFACE ${otns_dir}/ot-rfsim/src/event-sim.c)
  target_sources(native_simulator INTERFACE ${otns_drv_dir}/event_sim_ref.c)
  target_sources(native_simulator INTERFACE ${otns_drv_dir}/ieee802154_frame_parse.c)
  target_sources(native_simulator INTERFACE ${otns_drv_dir}/alarm_isr_bottom.c)

endif()
