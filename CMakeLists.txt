########################################################
# Enable Qt automatic features
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

########################################################
# Source files
set(buildingeditmanager_SRCS
  buildingeditmanager.cpp
  heightedit.cpp
  threedviewtool.cpp
  pointedit.cpp
  createtool.cpp
)

########################################################
# UI files
set(buildingeditmanager_UIS
  heightedit.ui
  threedview.ui
  createtool.ui
)

########################################################
# Resource files
set(buildingeditmanager_RCCS
  building_edit.qrc
)

########################################################
set(CMAKE_PREFIX_PATH "D:/conda/envs/qgis/Library")

########################################################
# Find Qt5 packages including Qt3D modules
find_package(Qt5 REQUIRED COMPONENTS
    Core
    Gui
    Widgets
    3DCore
    3DRender
    3DInput
    3DExtras
)

find_package(OpenGL REQUIRED) 

########################################################
# Build the plugin
add_library(buildingeditmanager MODULE
  ${buildingeditmanager_SRCS}
  ${buildingeditmanager_UIS_H}     
  ${buildingeditmanager_RCC_SRCS}  
)

########################################################
# Require C++17
target_compile_features(buildingeditmanager PRIVATE cxx_std_17)

########################################################
# Include directories
include_directories(
  ${CMAKE_SOURCE_DIR}/src/plugins
  ${CMAKE_CURRENT_BINARY_DIR}      

    D:/conda/envs/qgis/Library/include/qt
    D:/conda/envs/qgis/Library/include/qt/Qt3DCore
    D:/conda/envs/qgis/Library/include/qt/Qt3DRender
    D:/conda/envs/qgis/Library/include/qt/Qt3DExtras
)

########################################################
target_link_libraries(buildingeditmanager
    qgis_core
    qgis_gui
    qgis_3d 
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    D:/conda/envs/qgis/Library/lib/Qt53DCore_conda.lib
    D:/conda/envs/qgis/Library/lib/Qt53DRender_conda.lib
    D:/conda/envs/qgis/Library/lib/Qt53DInput_conda.lib
    D:/conda/envs/qgis/Library/lib/Qt53DExtras_conda.lib
    ${OPENGL_gl_LIBRARY}
)

########################################################
install(TARGETS buildingeditmanager
  RUNTIME DESTINATION ${QGIS_PLUGIN_DIR}
  LIBRARY DESTINATION ${QGIS_PLUGIN_DIR}
)
