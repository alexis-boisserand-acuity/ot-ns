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
  zephyr_library_sources(${otns_drv_dir}/radio_cca.c)
  zephyr_library_sources_ifdef(CONFIG_OPENTHREAD_BORDER_ROUTING ${otns_drv_dir}/infra_if.c)

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

  zephyr_include_directories(${otns_drv_dir})
  zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)
  zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../ot-rfsim/src)

  target_compile_options(native_simulator BEFORE INTERFACE
    -I${otns_drv_dir}
    -I${CMAKE_CURRENT_SOURCE_DIR}/..
    -I${CMAKE_CURRENT_SOURCE_DIR}/../openthread/include
    -I${CMAKE_CURRENT_SOURCE_DIR}/../ot-rfsim/src
  )
  target_sources(native_simulator INTERFACE ${otns_drv_dir}/ieee802154_bottom.c)
  target_sources(native_simulator INTERFACE ${otns_drv_dir}/ieee802154_frame_parse.c)
  target_sources(native_simulator INTERFACE ${otns_drv_dir}/alarm_isr_bottom.c)
  target_sources(native_simulator INTERFACE ${otns_drv_dir}/cli_bottom.c)

endif()
