# 实验室端附加脚本

本目录是 1.2.0 插件的服务端代码快照。将目录中的 `scripts/`、`tests/` 和依赖文件
复制到已有 Superpoint Transformer 工程的对应位置；不要把整个 QGIS 插件放进 SPT 工程。
SPT 原源码、配置、已验证环境和 DALES 权重需要另外准备。

## 环境与启动

服务使用独立 Conda 环境，SPT 工作进程仍调用原来的预测环境 Python。
在 SPT 工程根目录安装服务依赖：

```bash
conda create -n czy_spt_service python=3.10
conda activate czy_spt_service
python -m pip install -r requirements-prediction-service.txt
```

原预测环境只需满足附加 I/O 依赖 `requirements-copc-buildings.txt`，不要重新安装其 torch/CUDA/FRNN。
在当前服务终端设置至少16位ASCII访问密钥，然后启动：

```bash
# 自行替换为你的密钥，并在插件中输入相同值。
export SPT_API_TOKEN='ReplaceWithYourOwnLongToken123456'
python -u scripts/prediction_service.py \
  --python /path/to/existing-spt-env/bin/python \
  --host 0.0.0.0 --port 18080
```

`--python` 必须替换为现有可用 SPT 环境的实际 Python 路径。
直接 IP 连接适用于网络可达的可信局域网/VPN；插件填写 `http://实验室IP:18080`，不填 `0.0.0.0`。
也可绑定 `127.0.0.1` 并由用户自行配置 SSH 转发。程序不会修改防火墙、SSH、驱动或系统服务。

## 核对工作进程

```bash
python scripts/prediction_service.py --version
/path/to/existing-spt-env/bin/python scripts/prediction_worker.py --version
```

此快照分别是 `roof-service-v3-20261106`、`roof-v3-20261106`。
默认权重放在 SPT 工程根目录，名为 `spt-2_dales.ckpt`，或通过服务启动参数指定。

## 数据与恢复

所有任务新增文件位于 SPT 工程的 `prediction_jobs/<UUID>/`，不会清空已有目标建筑或旧工程结果。
服务串行处理任务，保留上传、缓存、日志和失败记录，不自动清理历史文件。
最终 GeoPackage 中 `target` 保留上传快照已有要素并追加预测结果，
`predicted_buildings` 只包含本次新增建筑，用于 QGIS 中追加，避免重复原要素。
已完整发布的结果在校验编号、大小和 SHA256 后可恢复下载；不完整结果不会冒充成功。

屋顶高程保持原一米XY邻域、原序前16点、至少5邻点、最低Z以上2.5米、法线阈值0.90与均值规则。
批量计算和有上限的解压缓存不改变点序或采样分辨率，不减地面高程，不作垂直基准换算。
没有满足条件的屋顶点时留空。

合成测试需在独立测试环境提供 `httpx`、服务依赖和 I/O/几何依赖；不调用真实 SPT 模型。

```bash
python -m unittest discover -s tests -p test_prediction_service.py -v
python -m unittest discover -s tests -p test_roof_elevation.py -v
```
