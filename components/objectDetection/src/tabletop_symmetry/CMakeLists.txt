project( tabletop_symmetry )

# dt
add_subdirectory(dt)

# Symmetry
add_library( mindGapper mindGapper.cpp )
target_link_libraries( mindGapper dt ${OpenCV_LIBS} ${PCL_LIBRARIES} )
set_target_properties( mindGapper PROPERTIES 
		       ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
		       RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )
