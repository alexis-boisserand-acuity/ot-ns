# Copyright (c) 2026, The OpenThread Authors
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the
#    names of its contributors may be used to endorse or promote products
#    derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

if(CONFIG_IEEE802154_OTNS)

  set(otns_drv_dir ${CMAKE_CURRENT_SOURCE_DIR}/drivers/ieee802154)
  get_filename_component(otns_dir ${CMAKE_CURRENT_SOURCE_DIR}/.. ABSOLUTE)
  set(ot_dir ${otns_dir}/openthread)

  zephyr_library()
  zephyr_library_sources(${otns_drv_dir}/ieee802154.c)
  zephyr_library_sources(${otns_drv_dir}/ieee802154_frame.c)
  zephyr_library_sources(${otns_drv_dir}/ieee802154_frame_parse.c)
  zephyr_library_sources(${otns_drv_dir}/ieee802154_isr.c)
  zephyr_library_sources(${otns_drv_dir}/alarm_isr.c)
  zephyr_library_sources(${otns_drv_dir}/cli.c)
  zephyr_library_sources(${otns_drv_dir}/host_msg.c)
  zephyr_library_sources(${otns_drv_dir}/radio_cca.c)
  zephyr_library_sources(${ot_dir}/examples/platforms/utils/mac_frame.cpp)
  zephyr_library_sources_ifdef(CONFIG_OPENTHREAD_BORDER_ROUTING ${otns_dir}/ot-rfsim/src/infra_if.c)
  if(CONFIG_OPENTHREAD_BORDER_ROUTING AND CONFIG_OPENTHREAD_UDP_FORWARD)
    zephyr_library_sources(${otns_drv_dir}/host_forward.c)
  endif()

  set_source_files_properties(${ot_dir}/examples/platforms/utils/mac_frame.cpp
    PROPERTIES INCLUDE_DIRECTORIES "${ot_dir}/src;${ot_dir}/examples/platforms/utils")
  zephyr_library_include_directories(${ot_dir}/src/core)
  target_link_libraries(${ZEPHYR_CURRENT_LIBRARY} PRIVATE ot-config)

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
  target_sources(native_simulator INTERFACE ${otns_drv_dir}/event_sim_ref.c)
  target_sources(native_simulator INTERFACE ${otns_drv_dir}/ieee802154_frame_parse.c)
  target_sources(native_simulator INTERFACE ${otns_drv_dir}/alarm_isr_bottom.c)
  target_sources(native_simulator INTERFACE ${otns_dir}/ot-rfsim/src/event-sim.c)

endif()
