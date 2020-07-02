project(test)

cmake_minimum_required(VERSION 3.2)

# Settings.
# ---------
set(TARGET test CACHE TYPE STRING)
set(OPENSIM_INSTALL_DIR $ENV{OPENSIM_HOME}
        CACHE PATH "Top-level directory of OpenSim install")



# Find and hook up to OpenSim.
# ----------------------------
find_package(OpenSim REQUIRED PATHS "${OPENSIM_INSTALL_DIR}")
find_package(Simbody REQUIRED PATHS "${SIMBODY_INSTALL_DIR}")

# Configure this project.
# -----------------------
file(GLOB AFFERENT_SOURCE_FILES Afferent/*.h Afferent/*.cpp)
file(GLOB SOURCE_FILES test_redux.cpp)

add_executable(${TARGET} ${AFFERENT_SOURCE_FILES} ${SOURCE_FILES})

target_link_libraries(${TARGET} ${OpenSim_LIBRARIES} ${Simbody_LIBRARIES} osimCommon osimLepton)

# This block copies the additional files into the running directory
# For example vtp, obj files. Add to the end for more extentions
file(GLOB DATA_FILES *.vtp *.obj *.osim)
foreach(dataFile ${DATA_FILES})
    add_custom_command(
        TARGET ${TARGET}
        COMMAND ${CMAKE_COMMAND}
        ARGS -E copy
        ${dataFile}
        ${CMAKE_BINARY_DIR})
endforeach(dataFile)
