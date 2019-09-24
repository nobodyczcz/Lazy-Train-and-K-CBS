# CMakeList.txt : CMake project for CBSH-rect-cmake, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

# Add source to this project's executable.
add_executable (CBS-K "driver.cpp")

# TODO: Add tests and install targets if needed.
set(all
		flat_map_loader.h
		flat_map_loader.cpp
        agents_loader.h
        compute_heuristic.h
		ICBSNode.h
		ICBSSearch.h
		LLNode.h
		map_loader.h
		MDD.h
		RectangleReasoning.h
		SingleAgentICBS.h
		agents_loader.cpp
        compute_heuristic.cpp
		ICBSNode.cpp
		ICBSSearch.cpp
		LLNode.cpp
		map_loader.cpp
		MDD.cpp
		RectangleReasoning.cpp
		SingleAgentICBS.cpp
        )
add_library(others ${all})
target_link_libraries( CBS-K others )



# Google's sparsehash
if (WIN32)
    include_directories("C:/Users/czcz2/Google Drive/MIT/GCITR/boost_1_71_0")
	include_directories("C:/Users/czcz2/Google Drive/MIT/GCITR/boost_1_71_0/stage/lib")
	include_directories("C:/Users/czcz2/Google Drive/MIT/GCITR/sparsehash/src")
	include_directories("C:/Users/czcz2/Google Drive/MIT/GCITR/sparsehash/src/windows")
	include_directories("C:/Users/czcz2/Anaconda3/include")
	target_link_libraries(CBS-K "C:/Users/czcz2/Google Drive/MIT/GCITR/boost_1_71_0/stage/lib/libboost_program_options-vc141-mt-x32-1_71.lib" )
else()
	find_path(SPARSEHASH_INCLUDE_DIRS sparsehash/dense_hash_map)
	if (NOT SPARSEHASH_INCLUDE_DIRS)
		message(FATAL_ERROR "Could not find <sparsehash/dense_hash_map>.")
	endif ()
	include_directories(SYSTEM ${SPARSEHASH_INCLUDE_DIRS})

	find_package(PythonInterp 3.6 REQUIRED)

	# This goes after, since it uses PythonInterp as hint
	find_package(PythonLibs 3.6 REQUIRED)
	include_directories(${PYTHON_INCLUDE_DIRS})

	set(Boost_USE_STATIC_LIBS OFF) 
	set(Boost_USE_MULTITHREADED ON)  
	set(Boost_USE_STATIC_RUNTIME OFF) 
	find_package(Boost REQUIRED COMPONENTS system program_options python-py36 ) 

	if(Boost_FOUND)
		include_directories(${Boost_INCLUDE_DIRS})
		target_link_libraries(CBS-K ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})
	else()
		 message(FATAL_ERROR "Could not find boost.")
	endif()
endif()
