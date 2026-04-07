#include "heightedit.h"
#include <qgsvectorlayer.h>
#include <qgsproject.h>
#include <qgsfeatureiterator.h>
#include <qgsfeature.h>
#include <qgsgeometry.h>
#include <qgsrectangle.h>
#include <qgsmapmouseevent.h>
#include <QDebug>
#include <QMessageBox>

HeightEditTool::HeightEditTool( QgsMapCanvas *canvas )
  : QgsMapTool( canvas )
{
  setCursor( Qt::CrossCursor );
  createRubberBand();

  auto layers = QgsProject::instance()->layers<QgsVectorLayer *>();
  if ( !layers.isEmpty() )
    mActiveLayer = layers.first();

  startEditingLayer( mActiveLayer );
}

HeightEditTool::~HeightEditTool()
{
  clearRubberBand();
}

// ---------------- 鼠标选择 ----------------
void HeightEditTool::canvasPressEvent( QgsMapMouseEvent *e )
{
  if ( !mActiveLayer || e->button() != Qt::LeftButton )
    return;

  mStartPoint = toMapCoordinates( e->pos() );
  mStartScreenPoint = e->pos();
  mDragging = true;
  mIsBoxSelecting = false;
}

void HeightEditTool::canvasMoveEvent( QgsMapMouseEvent *e )
{
  if ( !mDragging || !mRubberBand )
    return;

  int dx = e->pos().x() - mStartScreenPoint.x();
  int dy = e->pos().y() - mStartScreenPoint.y();
  if ( dx * dx + dy * dy < 25 )
    return;

  mIsBoxSelecting = true;

  QgsPointXY cur = toMapCoordinates( e->pos() );
  mRubberBand->reset();
  mRubberBand->addPoint( mStartPoint, false );
  mRubberBand->addPoint( QgsPointXY( cur.x(), mStartPoint.y() ), false );
  mRubberBand->addPoint( cur, false );
  mRubberBand->addPoint( QgsPointXY( mStartPoint.x(), cur.y() ), false );
  mRubberBand->closePoints();
  mRubberBand->show();
}

void HeightEditTool::canvasReleaseEvent( QgsMapMouseEvent *e )
{
  if ( !mDragging || !mActiveLayer )
    return;

  QgsPointXY endPoint = toMapCoordinates( e->pos() );
  if ( mIsBoxSelecting )
    selectByRectangle( QgsRectangle( mStartPoint, endPoint ), mShiftPressed );
  else
    selectAtPoint( endPoint, e->modifiers() & Qt::ShiftModifier );

  clearRubberBand();
  mDragging = false;
  mIsBoxSelecting = false;
}

// ---------------- 键盘事件 ----------------
void HeightEditTool::keyPressEvent( QKeyEvent *e )
{
  if ( e->key() == Qt::Key_Shift )
    mShiftPressed = true;
  QgsMapTool::keyPressEvent( e );
}

void HeightEditTool::keyReleaseEvent( QKeyEvent *e )
{
  if ( e->key() == Qt::Key_Shift )
    mShiftPressed = false;
  QgsMapTool::keyReleaseEvent( e );
}

// ---------------- 选择逻辑 ----------------
void HeightEditTool::selectAtPoint( const QgsPointXY &point, bool add )
{
  double r = searchRadiusMU( canvas() );
  QgsRectangle rect( point.x() - r, point.y() - r, point.x() + r, point.y() + r );

  QgsFeatureIterator it = mActiveLayer->getFeatures( QgsFeatureRequest( rect ) );
  QgsFeature feat;
  QgsFeatureId hit = -1;
  double best = r;

  while ( it.nextFeature( feat ) )
  {
    if ( !feat.hasGeometry() )
      continue;
    double d = feat.geometry().distance( QgsGeometry::fromPointXY( point ) );
    if ( d < best )
    {
      best = d;
      hit = feat.id();
    }
  }

  if ( hit != -1 )
  {
    QgsFeatureIds ids = add ? mActiveLayer->selectedFeatureIds() : QgsFeatureIds();
    if ( add )
    {
      if ( ids.contains( hit ) )
        ids.remove( hit );
      else
        ids.insert( hit );
    }
    else
    {
      ids.clear();
      ids.insert( hit );
    }
    mActiveLayer->selectByIds( ids );
    showSelectedAttributes();
  }
  canvas()->refresh();
}

void HeightEditTool::toggleSelectionAtPoint( const QgsPointXY &p )
{
  selectAtPoint( p, true );
}

void HeightEditTool::selectByRectangle( const QgsRectangle &rect, bool add )
{
  QgsFeatureIds ids = add ? mActiveLayer->selectedFeatureIds() : QgsFeatureIds();
  QgsFeatureIterator it = mActiveLayer->getFeatures( QgsFeatureRequest( rect ) );
  QgsFeature feat;
  while ( it.nextFeature( feat ) )
    ids.insert( feat.id() );
  mActiveLayer->selectByIds( ids );
  showSelectedAttributes();
  canvas()->refresh();
}

// ---------------- 橡皮筋 ----------------
void HeightEditTool::createRubberBand()
{
  if ( !mRubberBand )
  {
    mRubberBand = new QgsRubberBand( canvas() );
    mRubberBand->setColor( Qt::red );
    mRubberBand->setWidth( 1 );
    mRubberBand->hide();
  }
}

void HeightEditTool::clearRubberBand()
{
  if ( mRubberBand )
  {
    mRubberBand->hide();
    mRubberBand->reset();
  }
}

// ---------------- 弹出属性表 ----------------
void HeightEditTool::showSelectedAttributes()
{
  if ( !mActiveLayer )
    return;

  const QgsFeatureIds &fids = mActiveLayer->selectedFeatureIds();
  if ( fids.isEmpty() )
    return;

  if ( !mWidget )
  {
    mWidget = new QWidget();
    mUI.setupUi( mWidget );
    mWidget->setWindowTitle( "高度编辑" );
    mWidget->installEventFilter( this );

    // 滑块实时修改
    connect( mUI.horizontalSlider, &QSlider::valueChanged, this, &HeightEditTool::onSliderChanged );

    // comboBox字段选择
    connect( mUI.comboBox, &QComboBox::currentTextChanged, this, [this]( const QString &field ) {
      mHeightFieldName = field;
      initSliderCache(); // 重新缓存选中要素
    } );

    mUI.horizontalSlider->setMinimum( 0 );
    mUI.horizontalSlider->setMaximum( 5000 );
    mUI.horizontalSlider->setValue( 2500 );
  }

  // 填充 comboBox 字段
  mUI.comboBox->blockSignals( true );
  mUI.comboBox->clear();
  for ( const QgsField &f : mActiveLayer->fields() )
    mUI.comboBox->addItem( f.name() );
  mUI.comboBox->setCurrentText( mHeightFieldName );
  mUI.comboBox->blockSignals( false );

  // 构建属性表（只显示选中要素）
  QTableWidget *table = mUI.tableProperties;
  table->blockSignals( true );
  table->clear();
  int colCount = mActiveLayer->fields().count();
  table->setColumnCount( colCount );
  QStringList headers;
  for ( const QgsField &f : mActiveLayer->fields() )
    headers << f.name();
  table->setHorizontalHeaderLabels( headers );
  table->setRowCount( fids.size() );

  mInitialFieldValues.clear();
  int row = 0;
  for ( auto fid : fids )
  {
    QgsFeature feat;
    if ( !mActiveLayer->getFeatures( QgsFeatureRequest( fid ) ).nextFeature( feat ) )
      continue;

    mInitialFieldValues[fid] = feat.attribute( mHeightFieldName ).toDouble();

    for ( int c = 0; c < colCount; ++c )
    {
      table->setItem( row, c, new QTableWidgetItem( feat.attribute( c ).toString() ) );
    }
    ++row;
  }

  table->blockSignals( false );
  mWidget->show();
  mWidget->raise();
  mWidget->activateWindow();

  // ---------------- cellChanged 连接槽 ----------------
  connect( table, &QTableWidget::cellChanged, this, &HeightEditTool::onCellChanged );
}

// ---------------- 初始化滑块缓存 ----------------
void HeightEditTool::initSliderCache()
{
  if ( !mActiveLayer )
    return;

  const QgsFeatureIds &fids = mActiveLayer->selectedFeatureIds();
  if ( fids.isEmpty() )
    return;

  mInitialFieldValues.clear();

  double sum = 0.0;
  int count = 0;

  for ( auto fid : fids )
  {
    QgsFeature feat;
    if ( !mActiveLayer->getFeatures( QgsFeatureRequest( fid ) ).nextFeature( feat ) )
      continue;

    double h = feat.attribute( mHeightFieldName ).toDouble();
    mInitialFieldValues[fid] = h;

    sum += h;
    ++count;
  }

  // 以选中要素的“当前平均高度”作为参考高度
  mReferenceHeight = ( count > 0 ) ? ( sum / count ) : 0.0;

  // ---------- Slider 初始化（关键部分） ----------
  mUI.horizontalSlider->blockSignals( true );

  mUI.horizontalSlider->setMinimum( 0 );
  mUI.horizontalSlider->setMaximum(
    static_cast<int>( 500 * SLIDER_SCALE )
  );

  // ⚠️ 这里必须乘以 SLIDER_SCALE
  mUI.horizontalSlider->setValue(
    static_cast<int>( mReferenceHeight * SLIDER_SCALE )
  );

  mUI.horizontalSlider->blockSignals( false );
}

// ---------------- 滑块修改 ----------------
void HeightEditTool::onSliderChanged( int value )
{
  if ( !mActiveLayer || mInitialFieldValues.isEmpty() )
    return;

  double heightValue = value / SLIDER_SCALE;

  int idx = mActiveLayer->fields().indexOf( mHeightFieldName );
  if ( idx < 0 )
    return;

  mUI.tableProperties->blockSignals( true );

  int row = 0;
  for ( auto it = mInitialFieldValues.begin(); it != mInitialFieldValues.end(); ++it, ++row )
  {
    QgsFeatureId fid = it.key();

    QgsFeature feat;
    if ( mActiveLayer->getFeatures( QgsFeatureRequest( fid ) ).nextFeature( feat ) )
    {
      feat.setAttribute( idx, heightValue );
      mActiveLayer->updateFeature( feat );
    }

    auto *item = mUI.tableProperties->item( row, idx );
    if ( item )
      item->setText( QString::number( heightValue, 'f', 1 ) );
  }

  mUI.tableProperties->blockSignals( false );
}

// ---------------- 属性表修改立即保存 ----------------
void HeightEditTool::onCellChanged( int row, int column )
{
  if ( row < 0 || column < 0 || !mActiveLayer )
    return;

  auto fidIt = mInitialFieldValues.begin();
  std::advance( fidIt, row );
  QgsFeatureId fid = fidIt.key();

  double newValue = mUI.tableProperties->item( row, column )->text().toDouble();
  QgsFeature feat;
  if ( mActiveLayer->getFeatures( QgsFeatureRequest( fid ) ).nextFeature( feat ) )
  {
    feat.setAttribute( column, newValue );
    mActiveLayer->updateFeature( feat );
  }
}
// ---------------- 退出工具 ----------------
bool HeightEditTool::eventFilter( QObject *obj, QEvent *event )
{
  // 如果窗口不存在，交给父类处理
  if ( !mWidget )
    return QgsMapTool::eventFilter( obj, event );

  // 不处理 ESC，其他事件交给父类
  return QgsMapTool::eventFilter( obj, event );
}
// ---------------- 退出工具 ----------------
void HeightEditTool::startEditingLayer( QgsVectorLayer *layer )
{
  if ( !layer )
    return;
  if ( !layer->isEditable() )
    layer->startEditing();
}
