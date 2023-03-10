
SET(USE_VTK_VERSION "8.1" CACHE STRING "Expected VTK version")

SET_PROPERTY(CACHE USE_VTK_VERSION PROPERTY STRINGS 8.1)

#IF(MSVC_VERSION EQUAL 1900)
#    IF(USE_VTK_VERSION EQUAL 8.1)
#        FIND_PATH(VTK_INCLUDE_DIR NAMES vtk-8.1 HINTS ${CMAKE_SOURCE_DIR}/../../SDK/VTK_8.1.2/include)
#        FIND_PATH(VTK_LIB_DIR NAMES vtkCommonColor-8.1.lib HINTS ${CMAKE_SOURCE_DIR}/../../SDK/VTK_8.1.2/lib)
#    ENDIF()
#ENDIF()

IF(USE_VTK_VERSION EQUAL 8.1)
	FIND_PATH(VTK_INCLUDE_DIR NAMES vtk-8.1 HINTS ${CMAKE_SOURCE_DIR}/../../SDK/VTK_8.1.2/include)
	FIND_PATH(VTK_LIB_DIR NAMES vtkCommonColor-8.1.lib HINTS ${CMAKE_SOURCE_DIR}/../../SDK/VTK_8.1.2/lib)
ENDIF()

IF(USE_VTK_VERSION EQUAL 8.1)
  SET(VTK_LIBRARIES
    vtkCommonCore-8.1
    vtkCommonDataModel-8.1
    vtkCommonExecutionModel-8.1
    vtkCommonMath-8.1
    vtkCommonMisc-8.1
    vtkCommonSystem-8.1
    vtkCommonTransforms-8.1
    vtkFiltersCore-8.1
    vtkFiltersGeometry-8.1
    vtkFiltersGeneral-8.1
    vtkFiltersModeling-8.1
    vtkFiltersSources-8.1
    vtkFiltersTexture-8.1
    vtkglew-8.1
    vtkGUISupportQt-8.1
    vtkIOCore-8.1
    vtkIOImage-8.1
	vtkIOGeometry-8.1
	vtkIOLegacy-8.1
    vtkjpeg-8.1
    vtkpng-8.1
    vtkImagingColor-8.1
    vtkImagingCore-8.1
    vtkImagingHybrid-8.1
    vtkInteractionImage-8.1
    vtkInteractionStyle-8.1
    vtkInteractionWidgets-8.1
    vtkRenderingAnnotation-8.1
	vtkRenderingContext2D-8.1
	vtkRenderingContextOpenGL2-8.1
    vtkRenderingCore-8.1
    vtkRenderingFreeType-8.1
	vtkRenderingGL2PSOpenGL2-8.1
	vtkRenderingImage-8.1
	vtkRenderingLabel-8.1
	vtkRenderingLOD-8.1
	vtkRenderingQt-8.1
    vtkRenderingOpenGL2-8.1
    vtkRenderingVolume-8.1
    vtkRenderingVolumeOpenGL2-8.1
    vtksys-8.1
	vtkIOXML-8.1
	vtkIOPLY-8.1)

  SET(VTK_INCLUDE_DIRS ${VTK_INCLUDE_DIR}/vtk-8.1)
ENDIF()
