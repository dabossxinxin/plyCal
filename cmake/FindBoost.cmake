
SET(USE_Boost_VERSION "1.68" CACHE STRING "Expected Boost version")

SET_PROPERTY(CACHE USE_Boost_VERSION PROPERTY STRINGS 1.68)

#IF(MSVC_VERSION EQUAL 1900)
#    IF(USE_VTK_VERSION EQUAL 8.1)
#        FIND_PATH(VTK_INCLUDE_DIR NAMES vtk-8.1 HINTS ${CMAKE_SOURCE_DIR}/../../SDK/VTK_8.1.2/include)
#        FIND_PATH(VTK_LIB_DIR NAMES vtkCommonColor-8.1.lib HINTS ${CMAKE_SOURCE_DIR}/../../SDK/VTK_8.1.2/lib)
#    ENDIF()
#ENDIF()

IF(USE_Boost_VERSION EQUAL 1.68)
	FIND_PATH(Boost_INCLUDE_DIR NAMES boost-1_68 HINTS ${CMAKE_SOURCE_DIR}/../../SDK/Boost/include)
	FIND_PATH(Boost_LIB_DIR NAMES libboost_atomic-vc141-mt-gd-x64-1_68.lib HINTS ${CMAKE_SOURCE_DIR}/../../SDK/Boost/lib)
ENDIF()

IF(USE_Boost_VERSION EQUAL 1.68)
  SET(Boost_LIBRARIES
	libboost_atomic-vc141-mt
	libboost_chrono-vc141-mt
	libboost_container-vc141-mt
	libboost_context-vc141-mt
	libboost_contract-vc141-mt
	libboost_coroutine-vc141-mt
	libboost_date_time-vc141-mt
	libboost_exception-vc141-mt
	libboost_fiber-vc141-mt
	libboost_filesystem-vc141-mt
	libboost_graph-vc141-mt
	libboost_iostreams-vc141-mt
	libboost_locale-vc141-mt
	libboost_log-vc141-mt
	libboost_log_setup-vc141-mt
	libboost_math_c99-vc141-mt
	libboost_math_c99f-vc141-mt
	libboost_math_c99l-vc141-mt
	libboost_math_tr1-vc141-mt
	libboost_math_tr1f-vc141-mt
	libboost_math_tr1l-vc141-mt
	libboost_prg_exec_monitor-vc141-mt
	libboost_program_options-vc141-mt
	libboost_random-vc141-mt
	libboost_regex-vc141-mt
	libboost_serialization-vc141-mt
	libboost_signals-vc141-mt
	libboost_stacktrace_noop-vc141-mt
	libboost_stacktrace_windbg-vc141-mt
	libboost_stacktrace_windbg_cached-vc141-mt
	libboost_system-vc141-mt
	libboost_test_exec_monitor-vc141-mt
	libboost_thread-vc141-mt
	libboost_timer-vc141-mt
	libboost_type_erasure-vc141-mt
	libboost_unit_test_framework-vc141-mt
	libboost_wave-vc141-mt
	libboost_wserialization-vc141-mt
)

  SET(Boost_INCLUDE_DIRS ${Boost_INCLUDE_DIR}/boost-1_68)
ENDIF()
