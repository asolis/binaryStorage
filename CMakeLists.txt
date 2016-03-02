CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

SET(PROJECT_NAME "binstorage")
PROJECT("${PROJECT_NAME}")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# add opencv package to the project
FIND_PACKAGE( OpenCV REQUIRED )
#FIND_PACKAGE(HDF5)
find_package                (HDF5 QUIET COMPONENTS CXX)

if                          (HDF5_FOUND)

  include_directories       (SYSTEM ${HDF5_CXX_INCLUDE_DIR})

  add_library               (hdf5 STATIC IMPORTED)
  set_target_properties     (hdf5 PROPERTIES IMPORTED_LOCATION ${HDF5_hdf5_LIBRARY})
  add_library               (hdf5_cpp STATIC IMPORTED)
  set_target_properties     (hdf5_cpp PROPERTIES IMPORTED_LOCATION ${HDF5_hdf5_cpp_LIBRARY})
  set                       (HDF5_libraries     hdf5 hdf5_cpp)

endif                       (HDF5_FOUND)

INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS} )
#INCLUDE_DIRECTORIES( ${HDF5_INCLUDE_DIRS})
MESSAGE(STATUS "Project Name: ${PROJECT_NAME}")
MESSAGE(STATUS "    OpenCV version: ${OpenCV_VERSION} ")
MESSAGE(STATUS "    libraries: ${OpenCV_LIBS}")
MESSAGE(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
MESSAGE(STATUS "	hdf5: ${HDF5_CPP_LIBRARIES}")

FILE(GLOB files
  "*.h"
  "*.hpp"
  "*.cpp"
)
FILE(GLOB hidden
	".*"
)
FILE(GLOB resources
  "*.*"
)

ADD_EXECUTABLE(${PROJECT_NAME} ${files} )
TARGET_LINK_LIBRARIES( ${PROJECT_NAME} ${OpenCV_LIBS}  )
TARGET_LINK_LIBRARIES( ${PROJECT_NAME} ${HDF5_libraries} )

LIST(REMOVE_ITEM resources ${files} ${hidden} "${CMAKE_SOURCE_DIR}/CMakeLists.txt")
FILE(COPY ${resources} DESTINATION "Debug")
FILE(COPY ${resources} DESTINATION "Release")
