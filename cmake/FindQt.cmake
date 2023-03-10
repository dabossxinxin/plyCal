# 添加需要的Qt动态库
#IF(MSVC_VERSION EQUAL 2000)
#    FIND_PACKAGE(Qt5 REQUIRED COMPONENTS Core Gui Network Sql Xml Widgets
#        HINTS D:/Qt5.14.2/5.14.2/msvc2017_64/lib/cmake/Qt5/)
#ENDIF()

FIND_PACKAGE(Qt5 REQUIRED COMPONENTS Core Gui Network Sql Xml Widgets
	HINTS ${CMAKE_SOURCE_DIR}/../../SDK/Qt5.14.2/5.14.2/msvc2017_64/lib/cmake/Qt5/)
	