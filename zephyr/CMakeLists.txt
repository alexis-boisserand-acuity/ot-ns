# IEEE 802.15.4 OTNS out-of-source driver
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_IEEE802154_OTNS)

  set(otns_drv_dir ${CMAKE_CURRENT_SOURCE_DIR}/drivers/ieee802154)

  # Embedded (Zephyr CPU) side of the driver.
  zephyr_library()
  zephyr_library_sources(${otns_drv_dir}/ieee802154_otns.c)
  zephyr_include_directories(${otns_drv_dir})

  # Gives the embedded side access to ot-rfsim's radio.h/radio-parameters.h, so
  # timing and channel/threshold constants are reused rather than redefined
  # (see ieee802154_otns.c). Embedded-only: unlike this side, the runner side
  # below has no generated OpenThread config header to satisfy
  # <openthread/platform/radio.h>, which radio.h pulls in transitively.
  zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../ot-rfsim/src)

  # Runner ("bottom") side of the driver. This code is compiled as part of the
  # native simulator runner (host), so that it can use real POSIX sockets and
  # hook into the native simulator hardware-event scheduler for virtual-time
  # synchronization with OTNS.
  #
  # The runner side does not see the Zephyr Kconfig autoconf.h, so the driver
  # include path is passed explicitly.
  target_compile_options(native_simulator BEFORE INTERFACE
    -I${otns_drv_dir}
  )
  target_sources(native_simulator INTERFACE ${otns_drv_dir}/ieee802154_otns_bottom.c)

  if(CONFIG_IEEE802154_OTNS_CLI)

    # Embedded side: raw OpenThread CLI bridge over stdin/stdout, part of the driver.
    zephyr_library_sources(${otns_drv_dir}/otns_cli.c)

    # Runner side: owns stdin/stdout and redirects Zephyr output to stderr.
    target_sources(native_simulator INTERFACE ${otns_drv_dir}/otns_cli_bottom.c)

  endif()

endif()
