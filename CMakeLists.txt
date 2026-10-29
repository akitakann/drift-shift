cmake_minimum_required(VERSION 3.15)
project(racegame CXX)
set(CMAKE_CXX_STANDARD 17)

# ---------------------------------------------------------------------------
# 同じソースから 2 つのプラットフォーム向けにビルドできます。
#
#   ブラウザ版（Emscripten / WebAssembly）:
#     emcmake cmake -B build
#     cmake --build build
#
#   Linux ネイティブ版:
#     cmake -B build-native
#     cmake --build build-native
#
# emcmake を通してコンフィグすると EMSCRIPTEN 変数が立つので、それで分岐します。
# ---------------------------------------------------------------------------

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build type (Debug or Release)" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release")
endif()

message(STATUS "CMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}")

add_executable(racegame
    src/main.cpp
    src/track.cpp
    src/car.cpp
    src/audio.cpp
    src/trial.cpp
    src/render.cpp
)

set(ASSETS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/assets")

if(EMSCRIPTEN)
    message(STATUS "TARGET_PLATFORM=Browser (Emscripten)")
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/web/release-preset.json" RELEASE_PRESET)
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/web/bgm.js" BGM_PLAYER_SCRIPT)
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/web/release-preset.json"
        "${CMAKE_CURRENT_SOURCE_DIR}/web/bgm.js")
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/web/shell.html" "${CMAKE_CURRENT_BINARY_DIR}/release-shell.html" @ONLY)

    set(EMSCRIPTEN_SDL_FLAGS
        "-sUSE_SDL=2"
        "-sUSE_SDL_IMAGE=2"
        "-sSDL2_IMAGE_FORMATS=['png']"
        "-sUSE_SDL_MIXER=2"
        "-sSDL2_MIXER_FORMATS=['ogg']"
    )

    set(DEBUG_INFO_FORMAT "DWARF" CACHE STRING "Debug info format for Debug builds (DWARF or SOURCE_MAP)")
    set_property(CACHE DEBUG_INFO_FORMAT PROPERTY STRINGS "DWARF" "SOURCE_MAP")

    set(CMAKE_CXX_FLAGS_DEBUG "")

    if(DEBUG_INFO_FORMAT STREQUAL "SOURCE_MAP")
        set(DEBUG_INFO_COMPILE_FLAGS
            -gsource-map
            -fdebug-compilation-dir=.
            "-ffile-prefix-map=${CMAKE_SOURCE_DIR}/=../"
        )
        set(DEBUG_INFO_LINK_FLAGS -gsource-map)
    elseif(DEBUG_INFO_FORMAT STREQUAL "DWARF")
        set(DEBUG_INFO_COMPILE_FLAGS -g)
        set(DEBUG_INFO_LINK_FLAGS -g)
    else()
        message(FATAL_ERROR "DEBUG_INFO_FORMAT must be DWARF or SOURCE_MAP (got: ${DEBUG_INFO_FORMAT})")
    endif()

    message(STATUS "DEBUG_INFO_FORMAT=${DEBUG_INFO_FORMAT}")

    target_compile_options(racegame PRIVATE
        "$<$<CONFIG:Debug>:${DEBUG_INFO_COMPILE_FLAGS}>"
        $<$<CONFIG:Debug>:-O0>
        $<$<CONFIG:Release>:-O2>
        ${EMSCRIPTEN_SDL_FLAGS}
    )

    target_link_options(racegame PRIVATE
        "$<$<CONFIG:Debug>:${DEBUG_INFO_LINK_FLAGS}>"
        $<$<CONFIG:Debug>:-O0>
        $<$<CONFIG:Debug>:-sASSERTIONS=1>
        $<$<CONFIG:Release>:-O2>
        ${EMSCRIPTEN_SDL_FLAGS}
        "-sFORCE_FILESYSTEM=1"
        "-sALLOW_MEMORY_GROWTH=1"
        "-sUSE_WEBGL2=1"
        "-sMIN_WEBGL_VERSION=2"
        "-sMAX_WEBGL_VERSION=2"
        "-sFULL_ES3=1"
        "SHELL:--shell-file ${CMAKE_CURRENT_BINARY_DIR}/release-shell.html"
        "SHELL:--preload-file ${ASSETS_DIR}@/assets"
    )

    set_target_properties(racegame PROPERTIES OUTPUT_NAME "index" SUFFIX ".html")
    set_property(TARGET racegame APPEND PROPERTY LINK_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/web/shell.html")
    set_property(TARGET racegame APPEND PROPERTY LINK_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/web/release-preset.json")
    set_property(TARGET racegame APPEND PROPERTY LINK_DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/release-shell.html")
    set_property(TARGET racegame APPEND PROPERTY LINK_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/web/stamp_build.cmake")
    set_property(TARGET racegame APPEND PROPERTY LINK_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/web/music/slam-van.ogg"
        "${CMAKE_CURRENT_SOURCE_DIR}/web/music/time-bomb-remix.ogg")
    add_custom_command(TARGET racegame POST_BUILD
        COMMAND ${CMAKE_COMMAND} "-DINPUT=$<TARGET_FILE:racegame>" "-DSHELL_SOURCE=${CMAKE_CURRENT_SOURCE_DIR}/web/shell.html" -P "${CMAKE_CURRENT_SOURCE_DIR}/web/stamp_build.cmake"
        COMMENT "Keep the page, JavaScript and WebAssembly on one asset version"
    )
    # Stream full songs through browser audio, outside the preloaded Wasm data.
    add_custom_command(TARGET racegame POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_BINARY_DIR}/music"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${CMAKE_CURRENT_SOURCE_DIR}/web/music/slam-van.ogg"
                "${CMAKE_CURRENT_SOURCE_DIR}/web/music/time-bomb-remix.ogg"
                "${CMAKE_CURRENT_BINARY_DIR}/music/"
        COMMENT "Copy the two streamed BGM tracks"
    )

else()
    message(STATUS "TARGET_PLATFORM=Native (${CMAKE_SYSTEM_NAME})")

    find_package(PkgConfig REQUIRED)
    find_package(OpenGL REQUIRED)
    pkg_check_modules(SDL2 REQUIRED IMPORTED_TARGET sdl2)
    pkg_check_modules(SDL2_IMAGE REQUIRED IMPORTED_TARGET SDL2_image)
    pkg_check_modules(SDL2_MIXER REQUIRED IMPORTED_TARGET SDL2_mixer)

    target_link_libraries(racegame PRIVATE
        PkgConfig::SDL2
        PkgConfig::SDL2_IMAGE
        PkgConfig::SDL2_MIXER
        OpenGL::GL
    )

    target_compile_options(racegame PRIVATE
        $<$<CONFIG:Release>:-O2>
    )

    add_custom_command(TARGET racegame POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
                "${ASSETS_DIR}" "$<TARGET_FILE_DIR:racegame>/assets"
        COMMENT "assets/ を実行ファイルの隣にコピー"
    )

    install(TARGETS racegame DESTINATION .)
    install(DIRECTORY "${ASSETS_DIR}" DESTINATION .)

    set(CPACK_GENERATOR "TGZ")
    set(CPACK_PACKAGE_NAME "racegame")
    set(CPACK_PACKAGE_VERSION "0.1.0")
    set(CPACK_PACKAGE_FILE_NAME "racegame-linux")
    set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Race game (Linux native build)")
    include(CPack)

endif()

# Native checks exercise handling, experiment metrics and offline audio separately.
if(NOT EMSCRIPTEN)
    enable_testing()
    add_executable(drift-test tests/drift_test.cpp src/car.cpp src/track.cpp)
    target_include_directories(drift-test PRIVATE src)
    add_test(NAME handling COMMAND drift-test)
    add_executable(trial-test tests/trial_test.cpp src/trial.cpp src/track.cpp)
    target_include_directories(trial-test PRIVATE src)
    add_test(NAME trial COMMAND trial-test)
    add_executable(audio-test tests/audio_test.cpp)
    target_include_directories(audio-test PRIVATE src)
    target_link_libraries(audio-test PRIVATE PkgConfig::SDL2 PkgConfig::SDL2_MIXER)
    add_test(NAME audio COMMAND audio-test)
    add_executable(presentation-test tests/presentation_test.cpp src/car.cpp src/track.cpp)
    target_include_directories(presentation-test PRIVATE src)
    add_test(NAME presentation COMMAND presentation-test)
    add_executable(time-attack-test tests/time_attack_test.cpp src/car.cpp src/track.cpp)
    target_include_directories(time-attack-test PRIVATE src)
    add_test(NAME time-attack COMMAND time-attack-test WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
    add_executable(vehicle-presentation-test tests/vehicle_presentation_test.cpp src/car.cpp src/track.cpp)
    target_include_directories(vehicle-presentation-test PRIVATE src)
    add_test(NAME vehicle-presentation COMMAND vehicle-presentation-test)
endif()
