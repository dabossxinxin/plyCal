set(CMAKE_INCLUDE_CURRENT_DIR ON)
# init_qt: Let's do the CMake job for us
set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTORCC ON) # Resource files
set(CMAKE_AUTOUIC ON) # UI file

file(GLOB UI_Files "*.ui")
file(GLOB UI_Source "*.cpp")
file(GLOB UI_Header "*.h")

qt5_wrap_ui(UI_Wraps ${UI_Files})

include_directories(${VTK_INCLUDE_DIRS})

# 分别存放.cpp、.h、.ui以及由ui文件生成的.h文件
source_group("Source" FILES ${UI_Source})
source_group("Header" FILES ${UI_Header})
source_group("Interface" FILES ${UI_Files})
source_group("Generate" FILES ${UI_Wraps})

add_executable(PlyCal_qt ${UI_Source} ${UI_Header} ${UI_Wraps})

target_link_libraries(PlyCal_qt ${LIB_NAME})
target_link_libraries(PlyCal_qt Qt5::Widgets Qt5::Core Qt5::Gui)

foreach(_var ${VTK_LIBRARIES})
    target_link_libraries(PlyCal_qt debug ${VTK_LIB_DIR}/${_var}d.lib
        optimized ${VTK_LIB_DIR}/${_var}.lib)
endforeach()
