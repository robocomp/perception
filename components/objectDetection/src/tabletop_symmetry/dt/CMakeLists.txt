# dt (Distance Transform code by Pedro Fzen...
project( dt )

add_library( dt dt.cpp )
target_link_libraries( dt ${OpenCV_LIBS} )
set_target_properties( dt PROPERTIES 
		       ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
		       RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )
