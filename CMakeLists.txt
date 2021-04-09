project( MiindOpenSim )
cmake_minimum_required(VERSION 2.6)

set(MIIND_INCLUDE_DIR "" CACHE PATH "MIIND Include directory")
set(MIIND_LIB_DIR "" CACHE PATH "MIIND library directory")
set(OPENSIM_INCLUDE_DIR "" CACHE PATH "OpenSim Include directory")
set(OPENSIM_DEP_BASE_DIR "" CACHE PATH "OpenSim Dependencies Intalled directory")
set(OPENSIM_LIB_DIR "" CACHE PATH "OpenSim library directory")
set(OPENSIM_DEP_LIB_DIR "" CACHE PATH "OpenSim Dependencies SimBody library directory")


include_directories( ${MIIND_INCLUDE_DIR} )
include_directories( ${OPENSIM_INCLUDE_DIR} )
include_directories( ${OPENSIM_INCLUDE_DIR}/OpenSim )
include_directories( ${OPENSIM_DEP_BASE_DIR}/simbody/include )
include_directories( ${OPENSIM_DEP_BASE_DIR}/spdlog/include )

set ( LIBLIST
    ${MIIND_LIB_DIR}/miindgeom.lib
    ${MIIND_LIB_DIR}/miindmpi.lib
	${MIIND_LIB_DIR}/miindnumtools.lib
	${MIIND_LIB_DIR}/miindtwod.lib
	${OPENSIM_LIB_DIR}/osimTools.lib
	${OPENSIM_LIB_DIR}/osimActuators.lib
	${OPENSIM_LIB_DIR}/osimAnalyses.lib
	${OPENSIM_LIB_DIR}/osimCommon.lib
	${OPENSIM_LIB_DIR}/osimExampleComponents.lib
	${OPENSIM_LIB_DIR}/osimLepton.lib
	${OPENSIM_LIB_DIR}/osimSimulation.lib
	${OPENSIM_DEP_LIB_DIR}/SimTKsimbody.lib
	${OPENSIM_DEP_LIB_DIR}/SimTKmath.lib
	${OPENSIM_DEP_LIB_DIR}/SimTKcommon.lib
)

set ( SOURCELIST
Afferent/Millard12EqMuscleWithAfferents.cpp
Afferent/Lin02GolgiTendonOrgan.cpp
Afferent/Mileusnic06Spindle.cpp
NeuralController.cpp
arm.cpp
)

set ( HEADERLIST
Afferent/Millard12EqMuscleWithAfferents.h
Afferent/Lin02GolgiTendonOrgan.h
Afferent/Mileusnic06Spindle.h
NeuralController.h
)

add_executable(MiindOpenSim ${HEADERLIST} ${SOURCELIST})
target_link_libraries(MiindOpenSim  ${LIBLIST})