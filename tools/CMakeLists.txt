set (BUILD_TOOLS_agent_talker OFF CACHE BOOL "Build agent_talker")
set (BUILD_TOOLS_ngnet_generator ON CACHE BOOL "Build ngnet_generator")

if (BUILD_TOOLS_agent_talker)
  add_subdirectory(agent_talker)
endif()

if (BUILD_TOOLS_ngnet_generator)
  add_subdirectory(ngnet-generator)
endif()
