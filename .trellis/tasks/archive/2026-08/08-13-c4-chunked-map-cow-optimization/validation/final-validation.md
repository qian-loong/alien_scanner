# C4.1 最终验证记录

> 日期：2026-08-14
> 结论：功能与语义回归通过；Gate B 性能决策仍为 no-go，生产默认保持 `Vector`。

## 1. 构建与范围

- 构建类型：`RelWithDebInfo`。
- 受影响包：`perception_map_update`、`perception_local_map`、
  `perception_profiling`、`swarm_data_plane`。
- 完整 SHA、workload、短 A/B 指标和原始证据索引见
  `validation/gate-b-short-ab.md`。

## 2. 测试结果

三包回归与 C4 cave visual launch test 已完成；随后单独重跑
`perception_local_map` 的全部测试：

```bash
cd /workspaces/alien-scanner/ws
source /opt/ros/jazzy/setup.sh
source install/setup.sh
colcon test --packages-select perception_local_map \
  --event-handlers console_cohesion+
colcon test-result --verbose
```

结果：

```text
perception_local_map: 8/8 CTest items passed
376 tests, 0 errors, 0 failures, 0 skipped
```

覆盖 vector/chunked apply conformance、三维 replay、C4 route/resync/trust、
资源 runner 以及 cave visual 接线。短 A/B 四组均为 `valid=true`，362/362
消息应用、0 reject、最终 200000 cells。

## 3. 静态检查

```powershell
git diff --check
python -m py_compile ws/src/swarm_data_plane/test/run_c4_resource_profile.py
```

两项退出码均为 0。`git diff --check` 仅提示父任务 `task.json` 的既有
CRLF/LF 转换警告，没有 whitespace error。宿主与容器均未安装
`clang-format`，因此没有把独立格式检查记为通过；编译与测试使用现有 CMake
告警设置完成。

## 4. 未执行项与结论边界

- R5 的成熟三维 replay copied-cell P95 在 edge 8/16/32 下分别为
  9.50%/27.48%/36.69%，均未达到 `<5%`。
- flat SHA-256 仍完整遍历 canonical map；短 A/B 下 chunked 的端到端 apply
  未优于 vector，且常驻 PSS/USS 更高。
- 因 Gate B 已 no-go，没有启动 3 x 300 秒正式矩阵、Heaptrack、ASan/LSan
  或严格 Memcheck。这些项目是“未执行”，不是“通过”。
- chunked COW 保留为可选实现和后续研究基线；默认 storage 保持 `Vector`。
  Merkle/content identity 优化必须作为独立任务衡量。
