#include "buildingeditmanager.h"
#include "heightedit.h"
#include "threedviewtool.h"
#include "pointedit.h"
#include "createtool.h"
#include "view.h"

#include <QIcon>
#include <QMenu>
#include <QAction>

static const QString sName = QObject::tr( "Building Edit Manager" );
static const QString sDescription = QObject::tr( "Building editing tools" );
static const QString sCategory = QObject::tr( "Edit Tools" );
static const QString sPluginVersion = QObject::tr( "Version 0.1" );
static const QgisPlugin::PluginType sPluginType = QgisPlugin::UI;
static const QString sPluginIcon = QStringLiteral( "" );

// ========================================================================

BuildingEditManager::BuildingEditManager( QgisInterface *iface )
  : QgisPlugin( sName, sDescription, sCategory, sPluginVersion, sPluginType )
  , mIface( iface )
{
  QString projPath = "D:/conda/envs/qgis/Library/share/proj";
  qputenv( "PROJ_LIB", projPath.toUtf8() );
}

BuildingEditManager::~BuildingEditManager()
{
}

// ========================================================================
// 初始化 GUI
// ========================================================================

void BuildingEditManager::initGui()
{
  mMainAction = new QAction(
    QIcon( ":/building_edit/building_edit.png" ),
    tr( "Building Edit Manager" ),
    this
  );

  mMenu = new QMenu();
  mMainAction->setMenu( mMenu );

  // 高程编辑工具
  mHeightEditAction = new QAction( tr( "高程编辑" ), this );
  mHeightEditAction->setCheckable( true );
  mMenu->addAction( mHeightEditAction );

  connect( mHeightEditAction, &QAction::toggled, this, &BuildingEditManager::activateHeightEdit );

  // =========================================================
  // 【新增】：三维视图工具
  // =========================================================
  mThreeDViewAction = new QAction( tr( "三维视图" ), this );
  mThreeDViewAction->setCheckable( true );
  mMenu->addAction( mThreeDViewAction );

  connect(
    mThreeDViewAction,
    &QAction::toggled,
    this,
    &BuildingEditManager::activateThreeDView
  );

  // =========================================================
  // 【新增】点编辑工具
  // =========================================================
  mPointEditAction = new QAction( tr( "点编辑" ), this );
  mPointEditAction->setCheckable( true );
  mMenu->addAction( mPointEditAction );

  connect(
    mPointEditAction,
    &QAction::toggled,
    this,
    &BuildingEditManager::activatePointEdit
  );


  // =========================================================
  // 【新增】：新建/加点工具 (CreateTool)
  // =========================================================
  mCreateToolAction = new QAction( tr( "新建/加点" ), this );
  mCreateToolAction->setCheckable( true );
  mMenu->addAction( mCreateToolAction );

  connect(
    mCreateToolAction,
    &QAction::toggled,
    this,
    &BuildingEditManager::activateCreateTool
  );

  mIface->addToolBarIcon( mMainAction );
  mIface->addPluginToMenu( tr( "Building Edit Manager" ), mMainAction );

  // =========================================================
  // 【新增】：侧视图工具
  // =========================================================

  mViewAction = new QAction( tr( "侧视图" ), this );
  mViewAction->setCheckable( true );

  mMenu->addAction( mViewAction );

  connect(
    mViewAction,
    &QAction::toggled,
    this,
    &BuildingEditManager::activateView
  );
}

void BuildingEditManager::unload()
{
  mIface->removePluginMenu( tr( "Building Edit Manager" ), mMainAction );
  mIface->removeToolBarIcon( mMainAction );

  delete mMainAction;
  mMainAction = nullptr;
}

// ========================================================================
// 激活高程编辑工具
// ========================================================================

void BuildingEditManager::activateHeightEdit( bool checked )
{
  if ( mInternalSwitch )
    return;

  mInternalSwitch = true;

  if ( checked )
  {
    // 勾选高程编辑时，取消其他工具
    if ( mThreeDViewAction && mThreeDViewAction->isChecked() )
      mThreeDViewAction->setChecked( false );
    if ( mPointEditAction && mPointEditAction->isChecked() )
      mPointEditAction->setChecked( false );
    if ( mCreateToolAction && mCreateToolAction->isChecked() )
      mCreateToolAction->setChecked( false );

    if ( !mHeightEditTool )
      mHeightEditTool = new HeightEditTool( mIface->mapCanvas() );

    mIface->mapCanvas()->setMapTool( mHeightEditTool );
  }
  else
  {
    if ( mHeightEditTool )
      mIface->mapCanvas()->unsetMapTool( mHeightEditTool );
  }

  mInternalSwitch = false;
}

// ========================================================================
// 激活三维视图工具
// ========================================================================

void BuildingEditManager::activateThreeDView( bool checked )
{
  if ( checked )
  {
    // =================================================
    // 与高程编辑互斥
    // =================================================
    if ( mHeightEditAction && mHeightEditAction->isChecked() )
      mHeightEditAction->setChecked( false );
    if ( mPointEditAction && mPointEditAction->isChecked() )
      mPointEditAction->setChecked( false );
    if ( mCreateToolAction && mCreateToolAction->isChecked() )
      mCreateToolAction->setChecked( false );

    if ( !mThreeDViewTool )
      mThreeDViewTool = new ThreeDViewTool( mIface->mapCanvas(), mIface );

    mIface->mapCanvas()->setMapTool( mThreeDViewTool );
  }
  else
  {
    if ( mThreeDViewTool )
      mIface->mapCanvas()->unsetMapTool( mThreeDViewTool );
  }
}

// ========================================================================
// 激活点编辑工具
// ========================================================================

void BuildingEditManager::activatePointEdit( bool checked )
{
  if ( mInternalSwitch )
    return;

  mInternalSwitch = true;

  if ( checked )
  {
    if ( mHeightEditAction && mHeightEditAction->isChecked() )
      mHeightEditAction->setChecked( false );
    if ( mThreeDViewAction && mThreeDViewAction->isChecked() )
      mThreeDViewAction->setChecked( false );
    if ( mCreateToolAction && mCreateToolAction->isChecked() )
      mCreateToolAction->setChecked( false );

    if ( !mPointEditTool )
      mPointEditTool = new PointEdit( mIface->mapCanvas() );

    mIface->mapCanvas()->setMapTool( mPointEditTool );
  }
  else
  {
    if ( mPointEditTool )
      mIface->mapCanvas()->unsetMapTool( mPointEditTool );
  }

  mInternalSwitch = false;
}

// ========================================================================
// 激活新建/加点工具 (CreateTool)
// ========================================================================
void BuildingEditManager::activateCreateTool( bool checked )
{
  // 防止循环触发（如果你在函数内修改了 Action 的 Checked 状态）
  if ( mInternalSwitch )
    return;

  mInternalSwitch = true;

  if ( checked )
  {
    if ( mHeightEditAction && mHeightEditAction->isChecked() )
      mHeightEditAction->setChecked( false );
    if ( mThreeDViewAction && mThreeDViewAction->isChecked() )
      mThreeDViewAction->setChecked( false );
    if ( mPointEditAction && mPointEditAction->isChecked() )
      mPointEditAction->setChecked( false );

    // 2. 实例化工具（单例模式，避免重复创建）
    if ( !mCreateTool )
    {
      mCreateTool = new CreateTool( mIface->mapCanvas() );
    }

    // 3. 设置当前地图工具
    mIface->mapCanvas()->setMapTool( mCreateTool );
  }
  else
  {
    // 4. 取消勾选时，卸载工具
    if ( mCreateTool )
      mIface->mapCanvas()->unsetMapTool( mCreateTool );
  }

  mInternalSwitch = false;
}

// ========================================================================
// 激活侧视图工具
// ========================================================================

void BuildingEditManager::activateView( bool checked )
{
  if ( mInternalSwitch )
    return;

  mInternalSwitch = true;

  if ( checked )
  {
    if ( mHeightEditAction && mHeightEditAction->isChecked() )
      mHeightEditAction->setChecked( false );

    if ( mThreeDViewAction && mThreeDViewAction->isChecked() )
      mThreeDViewAction->setChecked( false );

    if ( mPointEditAction && mPointEditAction->isChecked() )
      mPointEditAction->setChecked( false );

    if ( mCreateToolAction && mCreateToolAction->isChecked() )
      mCreateToolAction->setChecked( false );

    if ( !mViewTool )
    {
      mViewTool = new View(
        mIface->mapCanvas(),
        mIface
      );
    }

    mIface->mapCanvas()->setMapTool(
      mViewTool
    );
  }
  else
  {
    if ( mViewTool )
      mIface->mapCanvas()->unsetMapTool(
        mViewTool
      );
  }

  mInternalSwitch = false;
}
// ========================================================================
// C 接口导出
// ========================================================================

QGISEXTERN QgisPlugin *classFactory( QgisInterface *iface )
{
  return new BuildingEditManager( iface );
}

QGISEXTERN const QString *name()
{
  return &sName;
}

QGISEXTERN const QString *description()
{
  return &sDescription;
}

QGISEXTERN int type()
{
  return sPluginType;
}

QGISEXTERN const QString *category()
{
  return &sCategory;
}

QGISEXTERN const QString *version()
{
  return &sPluginVersion;
}

QGISEXTERN const QString *icon()
{
  return &sPluginIcon;
}

QGISEXTERN void unload( QgisPlugin *plugin )
{
  delete plugin;
}
