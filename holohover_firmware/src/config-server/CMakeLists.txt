# Add a custom command that produces version.cpp, plus
# a dummy output that's not actually produced, in order
# to force version.cmake to always be re-run before the build

if(NOT CMAKE_BUILD_EARLY_EXPANSION)
    ADD_CUSTOM_COMMAND(
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/version.c
            ${CMAKE_CURRENT_BINARY_DIR}/_version.c
        COMMAND ${CMAKE_COMMAND} -P
                ${CMAKE_CURRENT_SOURCE_DIR}/version.cmake)
endif()

idf_component_register(
    SRCS
        "config-server.c"
        ${CMAKE_CURRENT_BINARY_DIR}/version.c
    INCLUDE_DIRS
        "include"
    REQUIRES
        nvs-manager
        esp_http_server
    EMBED_TXTFILES
        "configserver.html"
        "style.css"
)
