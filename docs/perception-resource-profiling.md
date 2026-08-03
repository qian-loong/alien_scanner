# C1 感知输入性能与内存剖析

本文记录 2026-07-25 对 `perception_input_node` 的外部工具基线。测量对象是
安装后的单个输入节点进程；fixture publisher、CSV sink 和 profiler 均为独立
进程，不计入目标资源数字。本轮没有修改五个感知包的业务源码。

## 结论

> **指标可信度前提（2026-07-31 补注）**：本报告的 CPU 与延迟绝对值来自
> WSL2/LinuxKit 环境，实测存在约 **±30%** 的宿主争用污染（相同工作量的 CPU
> 时间可波动 30%），因此下列 CPU 百分比**不应按三位有效数字理解**。判定依据与
> 指标三分类见 `docs/performance-memory-testing-cookbook.md` §6。
> **本报告的结论不受影响**：CPU 余量结论建立在约 150 倍的量级差上，远超污染幅度；
> 内存（RSS/PSS/USS）、斜率、泄漏与功能计数属于载荷无关指标，完全可信。

- 固定 mixed workload 下，目标单核 CPU 均值约 `0.64%`（±30%），三轮 P95 为
  `2%/1%/1%`；`perf stat` 的 600 秒 task-clock 换算为约 `0.6%`。
- 三轮稳态 RSS 约 `32.26-32.45 MiB`，PSS 约 `12.94-13.18 MiB`，USS 约
  `10.70-11.11 MiB`，均低于 C1 的 `100 MiB` 参考线。
- 三轮 RSS 回归斜率为 `0/8.18/-4.08 KiB/min`；PSS 为
  `0/8.12/-4.02 KiB/min`，远低于 `1024 KiB/min`，不满足“连续三轮疑似
  持续增长”的判定。
- ASan 的 60 项测试和 60 秒 mixed smoke 无越界/UAF；目标进程 LSan smoke
  无泄漏摘要且正常退出。
- Memcheck 发现 `64 B definitely lost`，栈位于 glibc 动态加载器与
  `rcutils_load_shared_library`，没有经过感知业务源码。保留为 ROS/动态加载器
  退出期 finding，不据此修改业务实现。
- CPU 热点主要是 3D 点搬运和 DDS/CDR 序列化；当前 CPU 余量很大，不建议在
  C2 前为这些热点做无证据优化。

## 环境与构建

| 项目 | 值 |
| --- | --- |
| OS / ROS | Ubuntu 24.04.4 / ROS 2 Jazzy |
| Kernel | Linux `6.10.14-linuxkit` |
| Compiler | GCC 13.3.0 |
| RMW | `rmw_fastrtps_cpp` |
| CPU / cgroup | 22 logical CPUs；无 CPU quota |
| Affinity | target CPU 0；fixture/sink CPU 1 |
| `perf_event_paranoid` | 2 |
| 容器 capability | `SYS_PTRACE`；当前容器未重建，尚无 `PERFMON` |
| Seccomp | 当前容器 `Seccomp=0` |
| Source revision | `ab98c7f72f19c8425f0f1d593a9b287471eaf0bc`，dirty worktree |
| Image ID | `sha256:76321bde799cb94b4e38ce61feb9836b968b2ff875ae0899fa518ea154e77a13` |
| Profiling Build ID | `26c5dbb5db84db3f5b67917b5692249411244def` |
| Profiling SHA-256 | `03cca516a0cc0fdb6cb113f4e1862025cd7033e68eb81e3df3c0566304c60236` |
| ASan Build ID | `550f49fe383b1f962d105f3541b5065a7999e6c9` |

Profiling 构建位于
`/tmp/alien-perception-profile-20260725-c1-03/{build-rel,install-rel}`。实际
compile command 同时包含：

```text
-fno-omit-frame-pointer -O2 -g -DNDEBUG
```

目标 ELF 有 `.debug_info`、`.debug_line` 和 `.symtab`，五包功能测试为
`60 tests, 0 errors, 0 failures, 0 skipped`。ASan 使用独立 build/install，参数为
`-O1 -g -DNDEBUG -fno-omit-frame-pointer -fsanitize=address`。

## 固定 workload 与有效性门

配置文件为 `scripts/perception-profile-mixed.yaml`：

- front/rear：两路 181 点 2D `LaserScan`；
- top：一路 `16 x 360` 点标准多线 3D `PointCloud2`；
- 10 Hz 每传感器，总输出约 30 Hz；range 5 m，seed 17；
- 两节点均从同一 YAML 启动，运行后逐参数回读；
- 三个输入 topic 和输出 topic 均要求 endpoint 精确 `1 publisher / 1 subscriber`，
  域碰撞或并发 run 混入额外 endpoint 时拒绝测量；
- preflight sink 与正式 sink 分开，正式计数只取 `t0-t1` 行区间；
- 每轮要求总数不低于 `duration x 27`、每个 ID 不低于
  `duration x 9`。
- 正式窗口每秒检查 fixture、target、sink、profiler/sampler 的 PID、starttime、
  zombie 状态和角色约束；任一角色提前退出或 PID 被复用即判 invalid。
- target 固定 CPU 0；perf、pidstat、smem sampler 和 trace-control 固定 CPU 1。
  LTTng consumer daemon 是容器共享服务，不改写其全局 affinity，报告中显式披露。

所有正式 run 均达到计数门，且 manifest 中记录 PID 角色、目标 `/proc` 信息、
实际 profiler 角色/affinity、capability、工具版本、二进制 build-id/SHA、退出码和
原始文件 SHA-256。

旧 raw manifest 的 `image_id=f200...` 实际是完整 container ID（其前 12 位与
`container_id` 完全一致），不是镜像 ID。最终汇总通过宿主镜像清单纠正为上表的
`sha256:76321...e77a13`；runner 现强制宿主传入带 `sha256:` 的 64 位镜像 ID，
缺失或误传容器 ID会在正式窗口前判 invalid。

最终汇总复核后，runner 进一步把 perf stat 必需事件与 enabled/running、perf lost
samples、Heaptrack 明细/汇总段、Massif 峰值/最终 snapshot 纳入机器 gate，并记录
runner、workload YAML、趋势脚本自身 SHA-256；这些解析器的完整/缺失样例均已做
正负回归。该强化只改变报告验收和 provenance，不改变正式测量窗口；既有
`21/24/28/32` 原始报告中的对应字段已逐项交叉核对，未用新 gate 伪造一次重跑。

2026-07-26 的最终代码审核又补强了后续 run 的编排合同：fixture、sink、target、
profiler 和 sampler 均等待 `exec` 身份与独立 PGID 稳定并记录 `/proc` starttime；
信号前重新校验 PID/starttime/PGID；perf 通过带单调时钟的 enable/disable/stop ACK
正常落盘；ROS trace 保存实际 event list；Heaptrack、Massif、Memcheck 要求唯一且
能识别目标的完整产物；三轮分析要求三个不同的 300 秒 plain-sample run，并按
实际时长拒绝缺失的 pidstat/smem 样本。`normal_completion=true` 仅在无强制停止、
无角色退出失败时写入。上述是对未来复现 runner 的静态加固，**本次没有重新运行
任何正式 profiler**；本文数值仍来自 2026-07-25 的旧 runner 原始产物，并只在旧
产物实际已有的 ACK、PID、时长、计数、报告段和 SHA 清单字段上逐项交叉核对。

## CPU 与 ROS 调度

### perf stat

`21-perf-stat-600` 有效窗口 `600.077 s`，front/rear/top 各 6001 条：

| 事件 | 结果 |
| --- | ---: |
| task-clock | 3647.17 ms |
| 单核 CPU 占用 | 0.608% |
| context switches | 0 |
| CPU migrations | 0 |
| page faults | 180030 |
| cycles/instructions/branch/cache | LinuxKit PMU `not supported` |
| time enabled/running | 100% |

perf 以冻结的 15 个目标 TID attach，control FIFO 的 enable/disable ACK 均成功。
硬件事件不可用是当前 LinuxKit 限制，不能据此计算 IPC 或 cache miss rate。

> **2026-07-31 补注**：该限制已确认为**结构性**——
> `/sys/bus/event_source/devices/` 无 `cpu` PMU（仅
> `breakpoint/kprobe/msr/power/software/tracepoint/uprobe`），root 亦不可得，
> 与 `perf_event_paranoid` 无关。**不要再尝试采集或分析硬件事件**。
> 上表 `task-clock` 与"单核 CPU 占用"同属 CPU 时间指标，带约 ±30% 的
> 争用污染。详见 `docs/performance-memory-testing-cookbook.md` §6。

### perf record

`cpu-clock:u` 在 LinuxKit 上被内核节流。1200 秒 DWARF run
`22-perf-record-1200` 只有 696 samples，符号质量合格但未达到 1000 样本门；
固定周期探针 `30-perf-record-300` 也只有 237 samples。profiling build 已保留
frame pointer，因此用 `31-perf-record-fp-probe-60` 验证
`-F 9999 --call-graph fp` 后执行最终 run。

最终 `32-perf-record-fp-1500` 的同步窗口为 `1500.091 s`，得到精确
1337 samples、0 lost records，self-symbol unknown 为 84 samples（`6.28%`），
工作区符号 89 samples；front/rear/top 同窗计数均为 15001，enable/disable ACK
均成功。perf header 中首末样本跨度为 `1494.600 s`；有效性按 ACK 包围的正式
workload 窗口判定，不用首末样本间隔替代测量时长。

| 排名 | Self overhead | 符号 |
| ---: | ---: | --- |
| 1 | 13.54% | libc `__memmove_avx_unaligned_erms` |
| 2 | 12.19% | Fast CDR `serialize(double)` |
| 3 | 6.28% | geometry_msgs CDR serialize |
| 4 | 4.79% | libc `pthread_mutex_lock` |
| 5 | 3.89% | geometry_msgs serialized-size calculation |
| 6 | 3.07% | `PerceptionInputNode::publish_observation` |
| 7 | 2.17% | `PointCloud2Adapter::extract_cloud` |
| 8 | 2.09% | Fast DDS `__rmw_wait` |
| 9 | 1.72% | libc `pthread_mutex_unlock` |
| 10 | 1.35% | geometry_msgs Fast CDR `serialize(double)` PLT |

perf 6.8 会把四位数样本缩写为头部 `# Samples: 1K`。旧解析器因此曾把前一轮
完整采集误判为 invalid；修正后以 `--no-children` 明细 sample 列精确求和。
上述 `22`、`30` 探针均保留为失败历史，没有覆盖或冒充最终基线。

### ROS 2 tracing

`23-ros-trace-60` 有效窗口 `60.109 s`，front/rear/top 各 601 条。正式窗口内
逐秒检查 LTTng session，共 60 次且全部为 active。按目标 vpid 和窗口过滤：

| 类别 | Jazzy tracepoint | 计数 |
| --- | --- | ---: |
| callback | `ros2:callback_start/end` | 3726 |
| take | `ros2:rcl_take` / `ros2:rmw_take` | 3606 |
| publish | `ros2:rclcpp_publish` / `ros2:rcl_publish` | 3726 |

三类事件均非零；fixture 和 sink 事件仅留在 raw trace，不计入上述数字。

## 堆与内存错误

### Heaptrack

`24-heaptrack-300` 有效窗口 `300.082 s`，三路各 3000 条。全生命周期报告：

| 指标 | 结果 |
| --- | ---: |
| allocation calls | 356232（1137/s） |
| temporary allocations | 29990（95/s） |
| peak heap | Heaptrack 原始值 7.29M |
| peak RSS including Heaptrack | Heaptrack 原始值 40.62M |
| Heaptrack total leaked 汇总 | 426.69 KiB |

报告的前 10 个高频分配栈及完整展开保存在该 run 的
`heaptrack-report.txt`；主要位于 Fast DDS、executor wait、字符串/向量复制和
消息接收/序列化路径。Heaptrack 的退出期 total leaked 汇总与目标 LSan 干净、
Memcheck 动态加载器 finding 交叉解释，不能单独证明业务持续泄漏。

### ASan 与 LSan

- `detect_leaks=0`：60 项测试全部通过；`25-asan-smoke-60` 为 `60.069 s`、
  三路各 600，
  无 AddressSanitizer error。
- `detect_leaks=1`：`26-lsan-smoke-60` 为 `60.094 s`、三路各 601，正常 exit 0，
  日志只有线程扫描信息，没有 LeakSanitizer error/summary。两个 smoke 都先验证
  目标 ELF 实际链接 `libasan`，避免误用普通构建形成假阴性。
- Python `launch_testing` 宿主会在退出时保留自身对象。测试通过
  `scripts/perception-profile-lsan.supp` 只抑制调用栈含 `python3.12` 的宿主泄漏；
  完整 60 项测试随后通过。共记录 10817 个宿主对象、4606363 B suppression；
  独立 C++ target/fixture 不加载 Python，因此目标泄漏不在该规则范围内。

### Memcheck 与 Massif

`27-memcheck-60` 有效窗口 `60.100 s`，三路各 598 条：

```text
definitely lost: 64 bytes in 1 block
indirectly lost: 0 bytes
possibly lost: 384 bytes in 1 block
still reachable: 201102 bytes in 220 blocks
```

唯一 definite 栈为 glibc `resize_scopes/dl_open` →
`rcutils_load_shared_library`，没有感知源码 frame。专用 exit code 42 表示完整 run
发现 finding，脚本保留完整摘要并仍将 profiler 执行本身标为有效。

`28-massif-180` 有效窗口 `180.113 s`，三路各 1801 条、110 snapshots。总峰值
`7506784 B`（7.159 MiB，snapshot 4），最终 `7476688 B`（7.130 MiB，
snapshot 109）；启动后曲线进入平台区，后续在消息周期内波动，没有持续爬升。

## 三次稳态基线

每轮 300 秒；排除前 60 秒后，pidstat 有 239 个稳态样本，smem/smaps 有 23 个
稳态样本。回归由 `scripts/analyze-perception-profile.py` 计算。

| Run | CPU mean / P95 | RSS mean | PSS mean | USS mean | RSS slope | PSS slope |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 29a | 0.644% / 2.0% | 32.27 MiB | 12.94 MiB | 10.70 MiB | 0 KiB/min | 0 KiB/min |
| 29b | 0.635% / 1.0% | 32.45 MiB | 13.18 MiB | 11.11 MiB | 8.18 KiB/min | 8.12 KiB/min |
| 29c | 0.639% / 1.0% | 32.26 MiB | 13.11 MiB | 11.07 MiB | -4.08 KiB/min | -4.02 KiB/min |

三轮 RSS/PSS 斜率均未超过 `1024 KiB/min`，结论为
`suspected_sustained_growth=false`。

## 工具版本

| 工具 | 版本 |
| --- | --- |
| perf | 6.8.12 |
| Heaptrack | 1.5.0 |
| Valgrind | 3.22.0 |
| LTTng | 2.13.11 |
| Babeltrace 2 | 2.0.5 |
| sysstat/pidstat | 12.6.1 |
| smem | 1.5-2 |

## 后续复现手册

以下命令用于后续代码、镜像或依赖变化后的重新采集。当前 runner 只支持本项目的
`perception_input_node`：目标可执行文件、`perception_fixtures`、mixed 参数、topic、
消息类型和 `sensor_id` 计数门均已写死，它不是可对任意 ROS 2 节点使用的通用
profiler runner。

本节是在 2026-07-26 补齐的长期手册。**2026-07-25 的正式结果没有因为本次文档
更新而重跑**；上文结果仍来自当日保存在容器 `/tmp` 中的原始证据。本节命令只按
当前脚本和配置做了静态核对，本次没有执行 colcon 构建、测试或任何 profiler。

### 宿主与容器准备

**先保全旧证据。** 如果当前 `alien-scanner-dev` 的
`/tmp/alien-perception-profile-20260725-c1-03` 仍存在且需要后续复核，必须在执行
下面的 build/up 序列前将整个目录导出到仓库外，并按各 run 的 `sha256sum.txt`
核验。新镜像上的 `docker compose up -d` 可能重建容器，而容器 `/tmp` 不是 bind
mount 或命名卷；旧容器删除后，该目录无法从仓库恢复。例如先在 PowerShell 执行：

```powershell
$auditRoot = 'D:\alien-profile-audit'
New-Item -ItemType Directory -Force -Path $auditRoot | Out-Null
docker cp 'alien-scanner-dev:/tmp/alien-perception-profile-20260725-c1-03' $auditRoot
```

开发工具由 `.devcontainer/Dockerfile` 安装，`PERFMON`、`SYS_PTRACE` 和 seccomp
设置来自 `.devcontainer/docker-compose.yml`。在 Windows PowerShell 的仓库根目录
重建/启动容器，并从宿主查询镜像 ID：

```powershell
docker compose -f .devcontainer/docker-compose.yml build ros2
docker compose -f .devcontainer/docker-compose.yml up -d ros2

$profileImageId = docker image inspect alien-scanner-jazzy:latest --format '{{.Id}}'
if ($profileImageId -notmatch '^sha256:[0-9a-f]{64}$') {
    throw "invalid image ID: $profileImageId"
}
docker exec -it -e "ALIEN_PROFILE_IMAGE_ID=$profileImageId" alien-scanner-dev bash
```

不能用 `docker ps` 显示的容器 ID 代替镜像 ID。进入容器后准备 ROS 环境和唯一的
`/tmp` 根目录；后续命令都在同一个 shell 中执行：

```bash
set -euo pipefail
source /opt/ros/jazzy/setup.bash
cd /workspaces/alien-scanner

[[ "${ALIEN_PROFILE_IMAGE_ID}" =~ ^sha256:[0-9a-f]{64}$ ]]
PROFILE_ROOT="/tmp/alien-perception-profile-$(date -u +%Y%m%dT%H%M%SZ)-$$"
test ! -e "${PROFILE_ROOT}"
mkdir -p "${PROFILE_ROOT}/runs"
export PROFILE_ROOT
```

`profile-perception.sh` 会拒绝已经存在的第三个参数目录。因此每条采集命令都必须
使用新的 `${PROFILE_ROOT}/runs/<run-name>`；不要预先创建该叶子目录，也不要复用
失败 run 的目录。

### 独立 profiling 构建与测试

从 colcon 工作区构建依赖闭包。两个 `--packages-up-to` 根包会带出
`perception_interfaces`、`perception_core`、`perception_adapters`、
`perception_input_node` 和 `perception_fixtures` 五包：

```bash
cd /workspaces/alien-scanner/ws

colcon --log-base "${PROFILE_ROOT}/log-rel" build \
  --build-base "${PROFILE_ROOT}/build-rel" \
  --install-base "${PROFILE_ROOT}/install-rel" \
  --packages-up-to perception_input_node perception_fixtures \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_CXX_FLAGS=-fno-omit-frame-pointer \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

colcon --log-base "${PROFILE_ROOT}/log-rel-test" test \
  --build-base "${PROFILE_ROOT}/build-rel" \
  --install-base "${PROFILE_ROOT}/install-rel" \
  --packages-up-to perception_input_node perception_fixtures \
  --event-handlers console_direct+

colcon test-result \
  --test-result-base "${PROFILE_ROOT}/build-rel" \
  --all --verbose
```

`RelWithDebInfo` 提供 `-O2 -g -DNDEBUG`；单独的 `CMAKE_CXX_FLAGS` 保留 frame
pointer，供当前 `perf record --call-graph fp` 使用；compile commands 用于核对
实际编译选项。独立的 build/install/log 路径避免覆盖日常 `ws/build`、`ws/install`
和 `ws/log`，runner 最终只测 `${PROFILE_ROOT}/install-rel` 中的安装后 ELF。

### 独立 ASan/LSan 构建与测试

Sanitizer 共用一套专用 ASan 编译产物，但它的 `build-asan`、`install-asan` 和
`log-asan*` 与普通 profiling 产物完全分离。初始缓存文件统一设置
`-O1 -g -DNDEBUG -fno-omit-frame-pointer -fsanitize=address` 及链接参数：

```bash
cd /workspaces/alien-scanner/ws

colcon --log-base "${PROFILE_ROOT}/log-asan" build \
  --build-base "${PROFILE_ROOT}/build-asan" \
  --install-base "${PROFILE_ROOT}/install-asan" \
  --packages-up-to perception_input_node perception_fixtures \
  --cmake-args \
    -C /workspaces/alien-scanner/scripts/perception-profile-asan.cmake

ASAN_RUNTIME="$(gcc -print-file-name=libasan.so)"
CXX_RUNTIME="$(c++ -print-file-name=libstdc++.so)"
SANITIZER_PRELOAD="${ASAN_RUNTIME}:${CXX_RUNTIME}"

env LD_PRELOAD="${SANITIZER_PRELOAD}" \
  ASAN_OPTIONS='detect_leaks=0:halt_on_error=1:exitcode=22' \
  colcon --log-base "${PROFILE_ROOT}/log-asan-test" test \
    --build-base "${PROFILE_ROOT}/build-asan" \
    --install-base "${PROFILE_ROOT}/install-asan" \
    --packages-up-to perception_input_node perception_fixtures \
    --event-handlers console_direct+

colcon test-result \
  --test-result-base "${PROFILE_ROOT}/build-asan" \
  --all --verbose

env LD_PRELOAD="${SANITIZER_PRELOAD}" \
  ASAN_OPTIONS='detect_leaks=1:halt_on_error=1:exitcode=23' \
  LSAN_OPTIONS='exitcode=23:log_threads=1:suppressions=/workspaces/alien-scanner/scripts/perception-profile-lsan.supp' \
  colcon --log-base "${PROFILE_ROOT}/log-lsan-test" test \
    --build-base "${PROFILE_ROOT}/build-asan" \
    --install-base "${PROFILE_ROOT}/install-asan" \
    --packages-up-to perception_input_node perception_fixtures \
    --event-handlers console_direct+

colcon test-result \
  --test-result-base "${PROFILE_ROOT}/build-asan" \
  --all --verbose
```

`LD_PRELOAD` 的顺序用于运行加载 ASan 化消息库的 Python `launch_testing` 宿主。
LSan suppression 只匹配 `python3.12` 宿主栈；独立 C++ target/fixture 不加载该
模块。测试必须先通过 `detect_leaks=0` 的 ASan 门，再执行 LSan 测试和下述 smoke。

### 九种采集模式

统一入口仍是四个位置参数：

```bash
scripts/profile-perception.sh \
  <mode> <install-prefix> <new-output-dir> <duration-seconds>
```

下表的“开发检查”只用于确认编排/工具可启动，不能形成正式基线。列出的短跑时长
参考 2026-07-25 旧 runner 的 smoke/probe 记录；当前加固 runner 未重新执行这些
命令，短跑也可能因正式证据门返回 invalid。Heaptrack、LSan 和 Massif 不建议另设
更短时长，直接使用正式时长。`perf-record` 虽在请求时长达到 120 秒后启用正式
样本门，但当前 LinuxKit 有采样节流，正式复现建议仍用 1500 秒以争取不少于
1000 samples。

| Mode | 开发检查 / 正式时长 | 工具与用途 |
| --- | --- | --- |
| `plain-sample` | 5 s / 300 s x 3 | `pidstat` 每秒采 CPU/RSS，`smem` 与 `smaps_rollup` 每 10 秒采 USS/PSS/RSS；用于稳态趋势 |
| `perf-stat` | 5 s / 600 s | `perf stat`；task-clock、调度、fault 及 PMU 事件可用性 |
| `perf-record` | 60 s symbol probe / 1500 s | `perf record` + frame-pointer call graph；样本、符号质量与热点 |
| `ros-trace` | 5 s / 60 s | `ros2 trace`、LTTng、Babeltrace 2；按目标 vpid 统计 callback/take/publish |
| `heaptrack` | 300 s / 300 s | Heaptrack；分配次数、临时分配、峰值堆、total leaked 汇总和调用栈 |
| `asan-smoke` | 5 s / 60 s | AddressSanitizer，`detect_leaks=0`；越界、UAF 等内存错误 |
| `lsan-smoke` | 60 s / 60 s | LeakSanitizer，`detect_leaks=1`；目标进程退出期泄漏 |
| `valgrind-memcheck` | 5 s / 60 s | Valgrind Memcheck；非法访问及 definite/indirect/reachable 分类 |
| `valgrind-massif` | 180 s / 180 s | Valgrind Massif + `ms_print`；含栈的峰值和时间线 |

正式命令逐项如下；九种工具应串行运行，ASan/LSan 两种 mode 必须使用
`install-asan`，其余 mode 使用 `install-rel`：

```bash
cd /workspaces/alien-scanner

scripts/profile-perception.sh perf-stat \
  "${PROFILE_ROOT}/install-rel" "${PROFILE_ROOT}/runs/perf-stat-600" 600
scripts/profile-perception.sh perf-record \
  "${PROFILE_ROOT}/install-rel" "${PROFILE_ROOT}/runs/perf-record-1500" 1500
scripts/profile-perception.sh ros-trace \
  "${PROFILE_ROOT}/install-rel" "${PROFILE_ROOT}/runs/ros-trace-60" 60
scripts/profile-perception.sh heaptrack \
  "${PROFILE_ROOT}/install-rel" "${PROFILE_ROOT}/runs/heaptrack-300" 300
scripts/profile-perception.sh asan-smoke \
  "${PROFILE_ROOT}/install-asan" "${PROFILE_ROOT}/runs/asan-smoke-60" 60
scripts/profile-perception.sh lsan-smoke \
  "${PROFILE_ROOT}/install-asan" "${PROFILE_ROOT}/runs/lsan-smoke-60" 60
scripts/profile-perception.sh valgrind-memcheck \
  "${PROFILE_ROOT}/install-rel" "${PROFILE_ROOT}/runs/memcheck-60" 60
scripts/profile-perception.sh valgrind-massif \
  "${PROFILE_ROOT}/install-rel" "${PROFILE_ROOT}/runs/massif-180" 180
# plain-sample 的三条正式命令见下一节。
```

每个成功 run 应同时满足 `run-manifest.txt` 中的 `valid=true` 和
`normal_completion=true`，并用同目录 `sha256sum.txt` 校验原始产物。工具 finding
和工具执行失败不是同一概念，例如完整 Memcheck 报告配合 exit code 42 是有效
finding。

### 三轮 plain-sample 与复算

正式趋势结论必须来自三个不同目录和三组独立 PID/单调时钟证据：

```bash
cd /workspaces/alien-scanner

scripts/profile-perception.sh plain-sample \
  "${PROFILE_ROOT}/install-rel" "${PROFILE_ROOT}/runs/plain-a-300" 300
scripts/profile-perception.sh plain-sample \
  "${PROFILE_ROOT}/install-rel" "${PROFILE_ROOT}/runs/plain-b-300" 300
scripts/profile-perception.sh plain-sample \
  "${PROFILE_ROOT}/install-rel" "${PROFILE_ROOT}/runs/plain-c-300" 300

python3 scripts/analyze-perception-profile.py \
  "${PROFILE_ROOT}/runs/plain-a-300" \
  "${PROFILE_ROOT}/runs/plain-b-300" \
  "${PROFILE_ROOT}/runs/plain-c-300" \
  --output "${PROFILE_ROOT}/plain-sample-analysis.json"
```

只复算本文 2026-07-25 结果、且旧容器原始目录仍存在时，使用：

```bash
python3 scripts/analyze-perception-profile.py \
  /tmp/alien-perception-profile-20260725-c1-03/runs/29a-plain-300 \
  /tmp/alien-perception-profile-20260725-c1-03/runs/29b-plain-300 \
  /tmp/alien-perception-profile-20260725-c1-03/runs/29c-plain-300
```

### 脚本与配置职责

| 文件 | 输入 / 使用方式 | 主要输出或产物 |
| --- | --- | --- |
| `scripts/profile-perception.sh` | `<mode> <install-prefix> <new-output-dir> <duration>`，并要求宿主镜像 ID 环境变量 | 每 run 的 manifest、PID/参数/graph/workload/退出状态、工具原始文件和报告、`sha256sum.txt` |
| `scripts/analyze-perception-profile.py` | 恰好三个有效且独立的 300 s `plain-sample` 目录；可选 `--output` | stdout JSON；可选 JSON 文件，含 CPU/RSS/PSS/USS、斜率和持续增长判定 |
| `scripts/test_analyze_perception_profile.py` | `unittest` 直接运行；内部生成临时合成 run/report | 正反例测试结果；覆盖趋势分析及 runner 内嵌 perf/Heaptrack/Massif/Memcheck 解析门 |
| `scripts/perception-profile-mixed.yaml` | runner 自动传给 fixture 和 input 两节点 | 固定 2D x 2 + 3D mixed workload；运行后参数 dump 和同窗消息计数证明其生效 |
| `scripts/perception-profile-asan.cmake` | colcon 的 `--cmake-args -C <file>` | 独立 sanitizer build/install 中的 ASan 化库和可执行文件、compile commands |
| `scripts/perception-profile-lsan.supp` | 仅通过 LSan 测试宿主的 `LSAN_OPTIONS=suppressions=<file>` 使用 | 抑制 Python 3.12 `launch_testing` 宿主栈；suppression 统计进入 sanitizer 输出 |

### 静态与合成回归

以下命令不启动 ROS 节点或 profiler；Python 用例会从 Bash runner 中提取内嵌
解析器，并用合成的正反例报告验证门禁：

```bash
bash -n scripts/profile-perception.sh
python3 -m unittest scripts/test_analyze_perception_profile.py -v
```

### 原始产物保留范围

大型 `perf.data`、Heaptrack、Massif 和 LTTng trace 必须只写入当前容器的
`/tmp/alien-perception-profile-<run-id>`，不写入 bind mount 的仓库，也不加入
Git。每个正式 run 有 `sha256sum.txt`；逐文件摘要以原始清单为准。容器被删除后
这些文件不再保证存在；需要长期审计时，应按 manifest 和 SHA 清单把完整证据集
导出到仓库之外的审计存储。

## 验收状态

- AC1 工具 version/smoke：通过。
- AC2 合规构建与 60 项功能测试：通过。
- AC3 perf stat：通过，硬件 PMU 事件明确 unavailable。
- AC4 perf record：端到端通过；完整采集、1337 samples、unknown 6.28%、0 lost。
- AC5 ROS trace：通过。
- AC6 Heaptrack：通过。
- AC7 ASan/LSan：目标节点通过；Python 宿主 suppression 有独立记录。
- AC8 Memcheck/Massif：工具运行通过；Memcheck 有 64 B 外部 finding。
- AC9 三次 plain sample：通过，无疑似持续增长。
- AC10 provenance、SHA、限制和 finding：通过。

当前证据支持进入 C2；建议保留 Memcheck 64 B finding，待 ROS Jazzy/glibc 或 RMW
版本变化后复测，不在 C1 业务代码内规避第三方动态加载器行为。
