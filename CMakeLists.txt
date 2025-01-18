cmake_minimum_required(VERSION 3.22)

if(NOT "${METAMODULE_SDK_DIR}" STREQUAL "")
	message("METAMODULE_SDK_DIR set by CMake variable ${METAMODULE_SDK_DIR}")
elseif (DEFINED ENV{METAMODULE_SDK_DIR})
    set(METAMODULE_SDK_DIR "$ENV{METAMODULE_SDK_DIR}")
	message("METAMODULE_SDK_DIR set by environment variable ${METAMODULE_SDK_DIR}")
else()
    set(METAMODULE_SDK_DIR "${CMAKE_CURRENT_LIST_DIR}/../metamodule-plugin-sdk")
	message("METAMODULE_SDK_DIR set to default: ${METAMODULE_SDK_DIR}")
endif()

include(${METAMODULE_SDK_DIR}/plugin.cmake)

project(CVfunkMetaModulePlugin 
    DESCRIPTION "CVfunk Plugin for MetaModule" 
    LANGUAGES   C CXX ASM
)

add_library(CVfunk STATIC)

set(SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR})

target_sources(CVfunk
    PRIVATE
    ${SOURCE_DIR}/src/Arrange.cpp
    ${SOURCE_DIR}/src/Collatz.cpp
    ${SOURCE_DIR}/src/Decima.cpp
    ${SOURCE_DIR}/src/EnvelopeArray.cpp
    ${SOURCE_DIR}/src/FlowerPatch.cpp
    ${SOURCE_DIR}/src/HexMod.cpp
    ${SOURCE_DIR}/src/ImpulseController.cpp
    ${SOURCE_DIR}/src/Magnets.cpp
    ${SOURCE_DIR}/src/Morta.cpp
    ${SOURCE_DIR}/src/Nona.cpp
    ${SOURCE_DIR}/src/Ouros.cpp
    ${SOURCE_DIR}/src/PentaSequencer.cpp
    ${SOURCE_DIR}/src/PreeeeeeeeeeessedDuck.cpp
	${SOURCE_DIR}/src/PressedDuck.cpp
	${SOURCE_DIR}/src/Ranges.cpp
	${SOURCE_DIR}/src/Signals.cpp
	${SOURCE_DIR}/src/Steps.cpp
	${SOURCE_DIR}/src/StepWave.cpp
	${SOURCE_DIR}/src/Steps.cpp
	${SOURCE_DIR}/src/Strings.cpp
	${SOURCE_DIR}/src/Syncro.cpp
	${SOURCE_DIR}/src/TriDelay.cpp
	${SOURCE_DIR}/src/plugin.cpp
)

target_include_directories(CVfunk PRIVATE 
	${SOURCE_DIR}/src 
)

set_property(TARGET CVfunk PROPERTY CXX_STANDARD 20)

if("${INSTALL_DIR}" STREQUAL "")
    set(INSTALL_DIR ${CMAKE_CURRENT_LIST_DIR}/metamodule-plugins)
endif()

create_plugin(
    SOURCE_LIB      CVfunk                                         # This is the cmake target name (defined in add_target)
    PLUGIN_NAME     CVfunk                                         # This must match the brand "slug" used in VCV Rack
    PLUGIN_JSON     ${SOURCE_DIR}/plugin.json                      # Path to the plugin.json file used by VCV Rack
    SOURCE_ASSETS   ${CMAKE_CURRENT_LIST_DIR}/assets               # Path to the assets/ dir containing the PNGs
    DESTINATION     ${INSTALL_DIR}  # Dir where plugin .mmplugin file should be put
)
