# *Try* to set compiler to gcc-6/g++-6 - not recommended way but the cleanest one; fallback to default compiler
find_program (GCC6_EXISTS gcc-6)
if (GCC6_EXISTS)
	message (STATUS "Forcing gcc-6 usage")
	set (CMAKE_C_COMPILER "${GCC6_EXISTS}")
else ()
	message (WARNING "gcc-6 not found, using default compiler")
endif ()
find_program (GPP6_EXISTS g++-6)
if (GPP6_EXISTS)
	message (STATUS "Forcing g++-6 usage")
	set (CMAKE_CXX_COMPILER "${GPP6_EXISTS}")
else ()
	message (WARNING "g++-6 not found, using default compiler")
endif ()
