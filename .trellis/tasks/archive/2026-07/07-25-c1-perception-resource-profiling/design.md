# C1 外部性能与内存剖析 - 技术设计

## 1. 分析边界

被测对象仅为 `perception_input_node` 进程。fixture publisher、CLI sink 和
profiler 在独立进程运行，固定到非目标 CPU。

```text
FixturePublisher ---- LaserScan x2 / PointCloud2 ---> PerceptionInputNode
                                                         |
                                                         v
                              ros2 topic echo sensor_data CSV sink/count
```

不通过修改业务节点增加计时或分配计数。业务源码在任务前后以路径 diff 核对。

## 2. 固定配置

新增 `scripts/perception-profile-mixed.yaml`，使用源码中的实际节点名和 ROS 2
参数层级。fixture 角表必须直接写弧度，不允许把度数传给 `_rad` 参数：

```yaml
perception_fixture_publisher:
  ros__parameters:
    mode: mixed
    scan_topic_front: fixture/scan/front
    scan_topic_rear: fixture/scan/rear
    cloud_topic: fixture/points
    scan_frame: fixture_scan_link
    cloud_frame: fixture_lidar_link
    publish_period_s: 0.1
    scan_point_count: 181
    cloud_azimuth_sample_count: 360
    cloud_range_m: 5.0
    elevation_angles_rad: [-0.2617993878, -0.2268928028, -0.1919862177,
      -0.1570796327, -0.1221730476, -0.0872664626, -0.0523598776,
      -0.0174532925, 0.0174532925, 0.0523598776, 0.0872664626,
      0.1221730476, 0.1570796327, 0.1919862177, 0.2268928028,
      0.2617993878]
    seed: 17

perception_input_node:
  ros__parameters:
    sensor_ids: [front, rear, top]
    requires_pose: false
    # 其余 contract 和 sensor.* 键逐项复制 sensor_descriptors_mixed.yaml。
```

两个可执行文件都必须使用 `--ros-args --params-file
scripts/perception-profile-mixed.yaml`。graph 验证前分别保存
`ros2 param dump /perception_fixture_publisher` 和
`ros2 param dump /perception_input_node`，并用 `ros2 param get` 机器校验上述
fixture 参数、input 的 `sensor_ids`、`requires_pose`、三类 type/topic/frame 和
输出 topic；值、类型或数组长度不符立即 invalid。

脚本将 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` 写入 manifest。22 CPU 环境默认
目标固定 CPU 0，fixture/sink 固定 CPU 1；CPU 数不足时脚本拒绝运行而不是改变
测量拓扑。

## 3. 构建和工具

Profiling 构建使用：

```text
-DCMAKE_BUILD_TYPE=RelWithDebInfo
-DCMAKE_CXX_FLAGS=-fno-omit-frame-pointer
-DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

其中 CMake 原生 `RelWithDebInfo` 提供 `-O2 -g -DNDEBUG`，独立的
`CMAKE_CXX_FLAGS` 追加 frame pointer；最终以 `compile_commands.json` 同时出现
四个选项为准，避免跨 Windows、Docker 和 shell 传递带空格的单一缓存参数时被
截断。

ASan/LSan 使用独立 build/install/log，并通过
`scripts/perception-profile-asan.cmake` 初始缓存统一增加 `-O1 -g -DNDEBUG
-fno-omit-frame-pointer -fsanitize=address` 编译参数及 address sanitizer 链接
参数，避免带空格的 flags 在 Windows、Docker 和 shell 边界被截断。

开发镜像增加 `heaptrack`、`valgrind`、`linux-tools-common`、
`linux-tools-generic`、`sysstat`、`smem`、`ros-jazzy-ros2trace`。Compose 增加
最小 `PERFMON` capability；若 Docker/LinuxKit 不支持该 capability，保留错误
证据并先验证 `perf_event_paranoid=2` 下的用户态 software event。

`perf` 解析顺序：

1. `/usr/bin/perf`；
2. wrapper 不支持 LinuxKit 版本时使用 `/usr/lib/linux-tools/*/perf` 最新文件；
3. 验证 stat/record 均支持 control FIFO/ACK，并保存短程 enable/disable smoke；
4. software event 失败或 control FIFO/ACK 不可用时，CPU profiling 阶段阻塞，
   不能用其他工具或近似时间窗冒充通过。

## 4. 统一脚本协议

`scripts/profile-perception.sh` 接受：

```text
<mode> <install-prefix> <output-dir> <duration-seconds>
```

支持 `plain-sample`、`perf-stat`、`perf-record`、`ros-trace`、`heaptrack`、
`asan-smoke`、`lsan-smoke`、`valgrind-memcheck`、`valgrind-massif`。脚本拒绝
已存在输出目录。

每次 run：

1. 创建唯一 ROS_DOMAIN_ID，记录环境、源码、镜像、工具、二进制 provenance；
   镜像 ID 必须由宿主以 `sha256:<64 hex>` 传入，拒绝容器 ID 或缺失值；
2. 以 workload YAML 启动 fixture，明确保存 fixture PID；
3. 以 sensor-data QoS 启动 preflight CSV sink，保存 PID；
4. 直接启动目标或按模式从进程启动插桩，所有命令显式传同一 YAML；
5. 按模式建立 PID 状态表并保存 `launcher_pid`、`tool_pid`、`tracee_pid`、`pgid`；
6. 回读关键参数并轮询 ROS graph，要求三个输入配对和输出 sink 配对均存在；
7. 预热 5 秒并确认 preflight CSV 已出现 front/rear/top，然后正常停止该 sink；
8. 启动新的正式 sink并等待配对；perf 以 disabled 状态按已校验的
   `tracee_pid` attach，或准备 ROS trace 的裁剪窗口；
9. perf 通过 control FIFO 收到 enable ACK 后记录单调时钟 `t0` 和 CSV 起始行
   偏移；其他模式在各自 profiler 已 ready 后记录同样的窗口起点；
10. 到时记录 `t1` 和 CSV 结束行偏移，再向 perf 发送 disable 并等待 ACK；只
    统计该内含于 profiler enabled 窗口的三类 sensor ID；
11. 按工具退出状态机正常结束目标和 profiler，再停止 sink、清理 fixture；
12. 分别解析应用、工具、tracer、sink 退出码和产物完整性，强杀即 invalid；
13. 生成 `run-manifest.txt` 与 raw artifact SHA-256；manifest 同时记录 runner、
    workload YAML 和趋势分析脚本自身的 SHA-256，并固定报告解析 locale。

sink 命令基线：

```bash
ros2 topic echo /perception/observations \
  perception_interfaces/msg/LidarObservation \
  --qos-profile sensor_data --csv --field sensor_id
```

CSV 起止偏移必须排除表头、Jazzy `--field` 输出的 `---` 分隔行、预热和清理期
行，数据字段只接受精确的 `front`、`rear`、`top`，其他未知非空值仍判 invalid。
有效 workload 门按 `t0-t1` 单调时钟实际秒数计算：总消息不少于
`duration*27`，每个 sensor ID 不少于 `duration*9`。

### 4.1 PID 状态表

- plain/ros-trace/perf：目标先直接启动，`$!` 即 `tracee_pid`；要求
  `/proc/<pid>/exe` 等于已记录 SHA 的安装后二进制。perf 另有 `tool_pid`，只按
  预热后冻结的 `/proc/<pid>/task/*` TID 列表 attach，不从 perf 的 `$!` 推断
  目标；结束时线程集合变化则 invalid，避免只采进程 leader 或漏掉既有 DDS 线程。
- Heaptrack：`launcher_pid/tool_pid` 为 controller；沿本 run 的精确父子树查找
  `/proc/exe` 唯一匹配目标 ELF 的后代作为 `tracee_pid`。无匹配或多匹配 invalid。
- Valgrind：记录 launcher PID、日志 `%p`、客户程序 cmdline 与 report 中目标 ELF；
  按 Valgrind 客户 PID 语义记录 `tracee_pid`，不要求 `/proc/exe` 指向目标 ELF。
- 每个模式在 `t0` 前保存 tracee 的 cmdline、PPid/NSpid、cgroup、affinity；资源
  采样和 vpid 过滤只接受该已验证角色。

### 4.2 退出状态机

- perf：固定 attach 窗口结束，先等待 perf 自行写完；随后只向 tracee 发送一次
  SIGINT并等待正常退出。perf 必须支持 control FIFO/ACK；以 disabled 状态启动，
  enable ACK 后设置 `t0`/起始偏移，设置 `t1`/结束偏移后 disable 并等待 ACK，
  随后 stop/等待落盘。不支持或 ACK 超时则 AC3/AC4 未完成。
- Heaptrack：只向唯一 tracee 发送 SIGINT，等待应用退出及 controller 自然落盘。
- Valgrind/Sanitizer：向客户进程语义对应 PID 发送一次 SIGINT，等待应用清理和
  工具摘要；禁止再向整个进程组重复广播同一信号。
- ROS trace：先通过 tracer 正常 stop/destroy 完成 trace，再正常结束目标。
- 所有模式最后停止正式 sink并等待 CSV 刷盘，再清理 fixture。任何角色进入
  SIGTERM/SIGKILL、超时或退出码/产物不符合该工具规则均 invalid。

## 5. 工具运行

| 顺序 | 工具 | 时长 | 有效性附加门 |
| --- | --- | ---: | --- |
| 1 | `perf stat` | 600 s | 预热后 attach；control ACK 同窗；完整 stat、enabled/running |
| 2 | `perf record` | 1500 s | 验收下限 120 s；control ACK 同窗；≥1000 samples、unknown≤20%、build-id |
| 3 | `ros2 trace` | 60 s | 目标 vpid 的 callback/take/publish 非零 |
| 4 | Heaptrack | 300 s | 正常退出、gz 与 print report 可解析 |
| 5 | ASan then LSan | tests + 60 s | ASan 先通过；LSan 独立正常退出 |
| 6 | Memcheck | 60 s | 正常退出、definite/indirect 分类 |
| 7 | Massif | 180 s | 正常退出、≥20 snapshots、peak 可解析 |
| 8 | plain sample ×3 | 300 s/run | pidstat/smem PID 正确、workload 达标 |

工具不并发。profiling 工具进程与 target 的关系必须通过 `/proc/<pid>/exe`、
命令行和工具 report 同时确认。

## 6. CPU 与符号化

- `perf stat` 尝试用户态 cycles/instructions/cache/branch 事件及 software event，
  所有 unsupported/not counted 单独列出。
- `perf record` 先 smoke 用户态 software event；当前 LinuxKit 上硬件 PMU 不可用，
  software sampling 也受到内核节流。1200 秒 DWARF run 仅得到 696 samples，
  300 秒固定周期探针仅得到 237 samples。profiling build 已保留 frame pointer，
  因此先以 60 秒探针验证 `cpu-clock:u -F 9999 --call-graph fp`，再运行 1500 秒
  最终窗口；最终得到 1337 samples。失败 run 作为探针历史保留，不降低 1000
  样本门。unknown 门按
  `--no-children` 的 self-symbol 计算；默认 children
  聚合的人工 `0xffffffffffffffff` 根节点不冒充采样 IP unknown，调用链另存独立
  报告。
- 保存 `perf buildid-list`、目标 `readelf -n`、SHA-256 和 `ldd`。
- stdio report 生成 `perf-quality.txt` 和 `perf-top10.txt`，记录采样数、lost
  samples、`--no-children` unknown 占比、工作区符号样本和按 overhead 排序的前 10 个
  symbol/DSO；正式时长
  （≥120 秒）不满足门即标记 invalid，补调试符号或延长采样后重跑。

## 7. ROS tracing

安装 `ros-jazzy-ros2trace` 后先保存 `ros2 trace --list`。trace 明确启用 rcl、
rclcpp 和 executor 相关 tracepoints，并记录 `vpid`/`procname` context。使用
`babeltrace2` 导出文本后，以脚本记录的目标 PID 和 `t0-t1` 过滤事件，按实际
列表建立以下类别映射：

```text
ros2:callback_start / ros2:callback_end
ros2:rcl_take or ros2:rmw_take
ros2:rclcpp_publish or ros2:rcl_publish
executor-related events when available
```

fixture/sink 事件可以保留在 raw trace，但不能计入目标汇总。若 Jazzy tracepoint
集合不含某类核心事件，报告实际列表并令 AC5 未完成；callback、take、publish
三类必须在目标 vpid 下分别非零，不能以其他类别存在代替。

## 8. 内存分析

- Heaptrack：默认完整分配追踪；`heaptrack_print` 保存 peak、leak、allocation、
  temporary allocation 和前 10 调用栈，并机器校验三个明细段与六项汇总字段。
- ASan：`detect_leaks=0:halt_on_error=1`；无错误后再运行 LSan：
  `detect_leaks=1:halt_on_error=1`。两个 smoke 均走统一 YAML、参数回读、graph 和
  正式 sink 门，日志与返回码分开。为 sanitizer 设置专用非零 error exitcode；
  干净 LSan run 可以没有原生泄漏摘要，以正常退出、workload 达标、日志无
  sanitizer error 且 error exitcode 未触发作为无泄漏证据。
- ASan 全量 launch 测试及 sanitizer smoke 的 Python sink 会加载 ASan 化的 ROS
  消息库，因此 Python 宿主显式按 `libasan:libstdc++` 顺序设置 `LD_PRELOAD`；
  fixture 关闭 leak 检测，避免把非目标进程的退出期分配混入目标结论。
- LSan 全量 launch 测试使用 `scripts/perception-profile-lsan.supp`，只抑制调用栈
  包含 `python3.12` 的测试宿主泄漏；独立 C++ target/fixture 不加载该模块，业务
  与 ROS C++ 栈仍受 leak gate 约束。报告同时保留未抑制探针结果和 suppression
  统计，不把 Python 宿主泄漏解释为目标节点泄漏。
- Memcheck：`--leak-check=full --show-leak-kinds=all
  --errors-for-leak-kinds=definite,indirect --track-origins=yes`；只把
  definite/indirect 和非法访问设为 finding。专用 error-exitcode 在 HEAP/LEAK/
  ERROR SUMMARY 完整时表示有效 finding；报告截断、Valgrind 自身失败或强杀才是
  invalid run。
- Massif：`--time-unit=ms --stacks=yes --detailed-freq=1 --max-snapshots=200`，
  用 `ms_print` 检查 60 秒后的稳态区和峰值，并从原始 snapshot 机器复算峰值与
  最终总内存。
- plain-sample：`pidstat -h -u -r -p <target> 1` 连续记录；每 10 秒用 `smem`
  核对目标 PID 的 USS/PSS/RSS，同时保存 `/proc/<pid>/smaps_rollup` 作为交叉证据。
- 三次 plain-sample 分别线性拟合 60–300 秒 PSS/RSS；只有三次均超过
  1 MiB/min 才标记疑似持续增长，再用 Heaptrack/Massif 调用栈解释。

## 9. 产物与范围

原始产物位于 `/tmp/alien-perception-profile-<run-id>`，每个子目录有 manifest、
退出状态、workload 计数、版本、provenance 和 SHA-256。仓库新增：

- `.devcontainer/Dockerfile` 与 `docker-compose.yml` 的开发工具/权限；
- `scripts/profile-perception.sh`、sanitizer 初始缓存与固定 YAML；
- `docs/perception-resource-profiling.md`；
- 当前任务文本记录。

不修改五个感知包的业务源码或公共接口，不提交 raw profiler 二进制。

## 10. 回滚和无效运行

- 工具安装失败：修复镜像包名后重试，不用自写计时器替代。
- perf software event 仍被拒绝：CPU profiling 保持未完成，记录 Docker/LinuxKit
  blocker，不扩大为 privileged 容器。
- 目标空载、参数回读不符、PID 角色有歧义、正式 sink 同窗计数不足、trace 任一
  核心类别无法按 vpid 过滤、工具非正常退出或产物不可解析：该 run 标记 invalid
  并重跑，不能进入汇总。LSan 干净 run 按 §8 的 error exitcode 规则判断，不
  强求不存在的零泄漏摘要。
- profiler 暴露业务缺陷：建立独立修复任务，不在 profiling 脚本绕过。
