# Building Edit Manager

QGIS C++/Qt 建筑编辑插件，包含轮廓编辑、屋顶模型与点云辅助功能。

当前版本：**1.2.0 — 加入机器学习预测功能**。

## 1.2.0 点云预测

在“新建 → 点云预测”中选择本机 COPC 点云和可写目标面图层，连接实验室服务执行
Superpoint Transformer（DALES 预训练权重）建筑语义分割、轮廓矢量化、直线拟合和小孔/小建筑清理。
可勾选高度初始化，计算屋顶代表高程。返回结果通过 SHA256 校验后追加到目标图层。

插件使用独立 HTTP 客户端直连配置的服务地址。服务地址与密钥在界面输入；密钥不保存到磁盘。
预测范围是整个所选 COPC 文件，不是当前地图显示范围。
如果原图层已经在编辑中，需要用户保存编辑；内存图层需另外保存到文件。

## 源码布局

- 根目录：QGIS C++ 插件、Qt UI、图标资源和 CMake 配置。
- `buildingprediction.*`：预测界面、图层快照和结果导入。
- `buildingpredictionclient.*`：鉴权、流式上传/下载、进度和任务恢复。
- `remote_prediction/`：配套实验室端附加脚本、依赖说明及合成测试。
- [CHANGELOG.md](CHANGELOG.md)：版本说明。

## 编译与部署

此插件按 QGIS 源码树内模块组织，应放入兼容 QGIS 源码树的 `src/plugins/building_edit_manager`，
并通过其 CMake 工程构建。现有 CMake 包含开发机的 Qt/Conda 路径，迁移构建机时需按实际依赖位置配置。
使用 `qgis_core`、`qgis_gui`、`qgis_3d`、Qt Widgets/Network 和 Qt3D，C++17。

远程服务部署参见 [remote_prediction/README.md](remote_prediction/README.md)。
附加脚本需要复制进已有且可正常运行的 Superpoint Transformer 工程，不能在本仓库中直接运行推理。
参考上游：[Superpoint Transformer](https://github.com/drprojects/superpoint_transformer)。

本仓库不包含模型权重、点云、账户密钥、实际任务目录和预测结果。
