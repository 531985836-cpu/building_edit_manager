#pragma once

#include "qgisplugin.h"
#include "qgisinterface.h"

#include <QObject>
#include <QAction>
#include <QMenu>
#include <QActionGroup>

class HeightEditTool;
class ThreeDViewTool;
class PointEdit;
class CreateTool;
class BuildingEditManager : public QObject, public QgisPlugin
{
    Q_OBJECT

  public:
    explicit BuildingEditManager( QgisInterface *iface );
    ~BuildingEditManager() override;

    void initGui() override;
    void unload() override;

  private slots:
    void activateHeightEdit( bool checked );
    void activateThreeDView( bool checked );
    void activatePointEdit( bool checked );
    void activateCreateTool( bool checked );

  private:
    QgisInterface *mIface = nullptr;

    QAction *mMainAction = nullptr;
    QMenu *mMenu = nullptr;
    QAction *mHeightEditAction = nullptr;

    HeightEditTool *mHeightEditTool = nullptr;

    QAction *mThreeDViewAction = nullptr;    
    ThreeDViewTool *mThreeDViewTool = nullptr;

    QAction *mPointEditAction = nullptr;
    PointEdit *mPointEditTool = nullptr;

    QAction *mCreateToolAction = nullptr;
    CreateTool *mCreateTool = nullptr;

    bool mInternalSwitch = false;
};
