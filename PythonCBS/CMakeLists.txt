# CMakeList.txt : CMake project for CBSH-rect-cmake, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

IF(NOT CMAKE_BUILD_TYPE)
  #SET(CMAKE_BUILD_TYPE "DEBUG")
  SET(CMAKE_BUILD_TYPE "RELEASE")
  #SET(CMAKE_BUILD_TYPE "RELWITHDEBINFO")
  #SET(CMAKE_BUILD_TYPE "MINSIZEREL")
ENDIF()

ADD_LIBRARY(PythonCBS SHARED
	PythonCBS.cpp
	PythonCBS.h
	../CBSH-rect-cmake/flat_map_loader.h
	../CBSH-rect-cmake/flat_map_loader.cpp
	../CBSH-rect-cmake/agents_loader.h
    ../CBSH-rect-cmake/compute_heuristic.h
	../CBSH-rect-cmake/ICBSNode.h
	../CBSH-rect-cmake/ICBSSearch.h
	../CBSH-rect-cmake/LLNode.h
	../CBSH-rect-cmake/map_loader.h
	../CBSH-rect-cmake/MDD.h
	../CBSH-rect-cmake/RectangleReasoning.h
	../CBSH-rect-cmake/SingleAgentICBS.h
	../CBSH-rect-cmake/agents_loader.cpp
   ../CBSH-rect-cmake/compute_heuristic.cpp
	../CBSH-rect-cmake/ICBSNode.cpp
	../CBSH-rect-cmake/ICBSSearch.cpp
	../CBSH-rect-cmake/LLNode.cpp
	../CBSH-rect-cmake/map_loader.cpp
	../CBSH-rect-cmake/MDD.cpp
	../CBSH-rect-cmake/RectangleReasoning.cpp
	../CBSH-rect-cmake/SingleAgentICBS.cpp
	../CBSH-rect-cmake/ReservationTable.h
	../CBSH-rect-cmake/ReservationTable.cpp
    
)



INCLUDE_DIRECTORIES("../CBSH-rect-cmake")

if (WIN32)
	set(BOOST_ROOT "C:/Users/czcz2/Google Drive/MIT/GCITR/boost_1_71_0")
	set(BOOST_LIBRARYDIR "C:/Users/czcz2/Google Drive/MIT/GCITR/boost_1_71_0/stage/lib")
	set(BOOST_INCLUDEDIR "C:/Users/czcz2/Google Drive/MIT/GCITR/boost_1_71_0/boost")

	include_directories("C:/Users/czcz2/Google Drive/MIT/GCITR/sparsehash/src")
	include_directories("C:/Users/czcz2/Google Drive/MIT/GCITR/sparsehash/src/windows")
	include_directories("C:/Users/czcz2/Anaconda3/include")
	include_directories("C:/Users/czcz2/Google Drive/MIT/GCITR/boost_1_71_0")
	include_directories("C:/Users/czcz2/Google Drive/MIT/GCITR/boost_1_71_0/stage/lib")
	
	
	FIND_PACKAGE(Boost)
	IF(Boost_FOUND)
		FIND_PACKAGE(Boost COMPONENTS system program_options python37)
		SET(Boost_USE_STATIC_LIBS OFF)
		SET(Boost_USE_MULTITHREADED ON)
		SET(Boost_USE_STATIC_RUNTIME OFF)
		include_directories(${Boost_INCLUDEDIR} )

		add_definitions(${Boost_LIB_DIAGNOSTIC_DEFINITIONS})
		TARGET_LINK_LIBRARIES(PythonCBS ${Boost_LIBRARIES})
		TARGET_LINK_LIBRARIES(PythonCBS "C:/Users/czcz2/Anaconda3/libs/python37.lib")


	ELSEIF(NOT Boost_FOUND)
	  MESSAGE(FATAL_ERROR "Unable to find correct Boost version. Did you set BOOST_ROOT?")
	ENDIF()
else()
	file(READ "/etc/issue" ETC_ISSUE)
	string(REGEX MATCH "Ubuntu|Manjaro" DIST ${ETC_ISSUE})
	if(DIST STREQUAL "Ubuntu")
		message(STATUS ">>>> Found Ubuntu <<<<")
		set(boostPython python-py36)
	elseif(DIST STREQUAL "Manjaro")
		message(STATUS ">>>> Found Manjaro <<<<")
		set(boostPython python37)
	else()
		message(STATUS ">>>> Found unknown distribution <<<<")
		set(boostPython python37)
	endif()

	find_path(SPARSEHASH_INCLUDE_DIRS sparsehash/dense_hash_map)
	if (NOT SPARSEHASH_INCLUDE_DIRS)
		message(FATAL_ERROR "Could not find <sparsehash/dense_hash_map>.")
	endif ()
	include_directories(SYSTEM ${SPARSEHASH_INCLUDE_DIRS} )

	find_package(PythonInterp 3.6 REQUIRED)

	# This goes after, since it uses PythonInterp as hint
	find_package(PythonLibs 3.6 REQUIRED)
	include_directories(${PYTHON_INCLUDE_DIRS})
	MESSAGE(${PYTHON_LIBRARIES} ${PYTHON_INCLUDE_DIRS})

	FIND_PACKAGE(Boost)
	IF(Boost_FOUND)
	  SET(Boost_USE_STATIC_LIBS OFF)
	  SET(Boost_USE_MULTITHREADED ON)
	  SET(Boost_USE_STATIC_RUNTIME OFF)
	  FIND_PACKAGE(Boost REQUIRED COMPONENTS system program_options ${boostPython})
	  INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

	  TARGET_LINK_LIBRARIES(PythonCBS ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})


	  

	ELSEIF(NOT Boost_FOUND)
	  MESSAGE(FATAL_ERROR "Unable to find correct Boost version. Did you set BOOST_ROOT?")
	ENDIF()

	

	include_directories(${PYTHON_INCLUDE_DIRS})
endif()

IF(CMAKE_COMPILER_IS_GNUCXX)
  ADD_DEFINITIONS("-Wall")
ENDIF()
