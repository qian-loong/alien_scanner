# C2 stage calibration build equivalence - 技术设计

## 1. 问题边界

正式问题不是“怎样扫描所有系统依赖”，而是证明下式中的唯一变化量是 stage target：

```text
unprobed:
  production local_map target + production dependencies + production helpers

callback/full:
  stage local_map target      + same production dependencies + same production helpers
```

旧 runner 只验证 target package，并把 target/helper 绑在同一 install base。stage prefix
因此重建了 helper 或从主工作区取得依赖。修复只扩展 C2 runner/helper/aggregator，不修改
C1 runner、C2 C++、CMakeLists 或消息契约。

## 2. 实际 workspace closure

production `--packages-up-to perception_profiling` 的九包顺序为：

```text
cave_world
perception_core
perception_interfaces
drone_scanner
perception_adapters
perception_input_node
perception_fixtures
perception_local_map
perception_profiling
```

comparison manifest 不复制 colcon dependency resolver。使用固定的 C2 domain 表：

| 集合 | 记录方式 | 原因 |
| --- | --- | --- |
| `perception_core` | cache/compile profile、全部 public headers、static archive | target 直接编译/静态链接依赖 |
| `perception_interfaces` | cache/compile profile、全部 generated headers、package libraries | target 直接编译/动态/生成代码依赖 |
| fixture/sink/oracle/YAML | production helper exact SHA/build ID | 覆盖 CPU 1 workload/evidence 角色及其静态算法闭包 |
| 其余六个 workspace packages | 不逐包写 manifest | formal target 不加载；helper exact identity 已覆盖其编入结果 |

固定集合若与 compile/link 证据发现的 workspace install 路径不一致即失败，不动态扩大集合。
这能暴露 C2 边界变化，同时避免演变为通用 SBOM。

## 3. Closure Artifact Contract

`local_map_build_provenance.py` 保留现有 target flag validation，并新增定向 capture。runner
写出 `target-workspace-closure.json`，schema version 为 1，概念结构如下：

```json
{
  "schema_version": 1,
  "paired_source_identity_sha256": "<64 hex>",
  "closure_install_base": "/tmp/.../install-production",
  "closure_build_base": "/tmp/.../build-production",
  "dependencies": [
    {
      "package": "perception_core",
      "prefix": "/tmp/.../install-production/perception_core",
      "build_type": "RelWithDebInfo",
      "compile_profile_sha256": "<64 hex>",
      "artifacts": [
        {
          "relative_path": "perception_core/lib/libperception_core.a",
          "kind": "static_library",
          "size": 4158378,
          "sha256": "<64 hex>",
          "build_id": null
        }
      ]
    },
    {
      "package": "perception_interfaces",
      "prefix": "/tmp/.../install-production/perception_interfaces",
      "build_type": "RelWithDebInfo",
      "compile_profile_sha256": "<64 hex>",
      "artifacts": []
    }
  ],
  "helpers": [
    {
      "name": "fixture",
      "relative_path": "perception_profiling/lib/perception_profiling/perception_profile_fixture",
      "sha256": "<64 hex>",
      "build_id": "<hex>"
    }
  ],
  "workload": {
    "relative_path": "perception_profiling/share/perception_profiling/config/profile_local_map.yaml",
    "sha256": "<64 hex>"
  },
  "dependency_comparison_sha256": "<64 hex>",
  "helper_set_sha256": "<64 hex>"
}
```

### 3.1 Canonicalization

- JSON UTF-8、LF、`sort_keys=True`、compact separators，并以一个 newline 结尾。
- dependencies 按 package 排序；artifacts/helpers 按 relative path 排序。
- public/generated header tree 逐文件列入 artifacts，symlink 先 `realpath` 后读取，但
  relative path 保留 install-base 视角。目录、socket 和 broken symlink 拒绝。
- `compile_profile_sha256` 对该 dependency package 的非测试 compile entries 做 token
  normalization 后计算；先验证 required/conflicting flags，再 hash。绝对 build directory
  不进入 comparison token，compiler 路径、source-relative path 和有效 flags 进入。
- `dependency_comparison_sha256` 覆盖 source identity、closure realpath、两个 dependency
  records；`helper_set_sha256` 覆盖三个 helper 和 YAML。完整 JSON 自身 SHA 由 raw
  `sha256sum.txt` 保存。

### 3.2 Run Manifest Fields

新增并在 calibration 中强制：

```text
paired_source_identity_sha256
workspace_closure_install_base
workspace_closure_build_base
workspace_closure_manifest_sha256
workspace_dependency_comparison_sha256
profiling_prefix
profiling_helper_set_sha256
```

unprobed/callback/full 的上述字段必须逐字相同。callback/full 继续要求 target prefix、
SHA 和 build ID 相同；unprobed target 必须不同且 stage option OFF。calibration output
schema 从 1 升为 2，避免新旧 validity contract 混淆。per-run analyzer 不升 schema，仍可
读取旧 raw；只有跨 run calibration aggregator 对缺字段 fail closed。

## 4. Runner Prefix And Resolution Contract

保留 CLI：

```text
profile-local-map.sh <mode> <target-install-base> <new-output-dir>
                     <duration> [workload] [callback|full]
```

新增 stage-only 环境输入：

```text
ALIEN_PROFILE_CLOSURE_INSTALL_BASE=/tmp/.../install-production
ALIEN_PROFILE_CLOSURE_BUILD_BASE=/tmp/.../build-production
ALIEN_PROFILE_PAIRED_SOURCE_IDENTITY=/tmp/.../paired-source-identity.txt
```

非 stage mode 默认 closure install/build base 等于 target install/build base，保持当前
行为。stage-latency 必须显式提供三个值，且 target install base 与 closure install base
不同。

stage source 顺序固定：

1. source `/opt/ros/jazzy/setup.bash` 的调用者环境；
2. runner source production closure `setup.bash`；
3. runner source stage target `local_setup.bash`，不 source overlay `setup.bash`；
4. `ros2 pkg prefix perception_local_map` 必须落在 target overlay；
5. `ros2 pkg prefix perception_profiling` 必须落在 production closure；
6. helper ELF/YAML、dependency CMake dirs、target compile/link/ldd 全部通过 closure validator。

runner 在 oracle 或 ROS role 启动前完成以上步骤。失败只创建 invalid raw 与明确 reason，
不进入 warmup。`install_prefix` 保留为 target install base，另写 `profiling_prefix` 和
closure fields，避免改变既有字段语义。

## 5. Paired Build Procedure

implementation 必须选择新的、不存在的 `ALIEN_C2_PAIR_ROOT`。以下是容器内的冻结
single-worker 命令形状；实际日期后缀在执行前记录到 task，不复用示例目录。

### 5.1 Production OFF closure

```bash
export ALIEN_C2_PAIR_ROOT=/tmp/alien-c2-stage-pair-YYYYMMDD
test ! -e "${ALIEN_C2_PAIR_ROOT}"

env -i \
  ALIEN_C2_PAIR_ROOT="${ALIEN_C2_PAIR_ROOT}" \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  LANG=C.UTF-8 LC_ALL=C.UTF-8 \
  /bin/bash --noprofile --norc -c '
set -eo pipefail
set +u
source /opt/ros/jazzy/setup.bash
set -u
cd /workspaces/alien-scanner/ws
export MAKEFLAGS=-j1
export CMAKE_BUILD_PARALLEL_LEVEL=1
colcon --log-base "${ALIEN_C2_PAIR_ROOT}/log-production" build \
  --executor sequential \
  --build-base "${ALIEN_C2_PAIR_ROOT}/build-production" \
  --install-base "${ALIEN_C2_PAIR_ROOT}/install-production" \
  --event-handlers console_direct+ \
  --packages-up-to perception_profiling \
  --cmake-force-configure \
  --cmake-args \
    -C /workspaces/alien-scanner/scripts/perception-profile-relwithdebinfo.cmake \
    -DPERCEPTION_LOCAL_MAP_ENABLE_STAGE_LATENCY_TRACEPOINTS=OFF
'
```

构建前生成 `paired-source-identity.txt`；构建后立即重算并要求相同。production 九包的
cache/compile/install evidence 必须全部解析到 pair root 或 `/opt/ros/jazzy`。

### 5.2 Stage ON local-map-only overlay

```bash
env -i \
  ALIEN_C2_PAIR_ROOT="${ALIEN_C2_PAIR_ROOT}" \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  LANG=C.UTF-8 LC_ALL=C.UTF-8 \
  /bin/bash --noprofile --norc -c '
set -eo pipefail
set +u
source /opt/ros/jazzy/setup.bash
source "${ALIEN_C2_PAIR_ROOT}/install-production/setup.bash"
set -u

for variable_name in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH \
  LD_LIBRARY_PATH PYTHONPATH; do
  value="${!variable_name-}"
  case ":${value}:" in
    *:/workspaces/alien-scanner/ws/install:* )
      echo "forbidden main-workspace install in ${variable_name}" >&2
      exit 1
      ;;
  esac
done

cd /workspaces/alien-scanner/ws
export MAKEFLAGS=-j1
export CMAKE_BUILD_PARALLEL_LEVEL=1
colcon --log-base "${ALIEN_C2_PAIR_ROOT}/log-stage" build \
  --executor sequential \
  --build-base "${ALIEN_C2_PAIR_ROOT}/build-stage" \
  --install-base "${ALIEN_C2_PAIR_ROOT}/install-stage" \
  --event-handlers console_direct+ \
  --packages-select perception_local_map \
  --cmake-force-configure \
  --cmake-args \
    -C /workspaces/alien-scanner/scripts/perception-profile-relwithdebinfo.cmake \
    -DPERCEPTION_LOCAL_MAP_ENABLE_STAGE_LATENCY_TRACEPOINTS=ON
'
```

stage build 后再次重算 source identity。若 colcon/CMake 解析到主 workspace install、
重建了任一 helper/dependency package、或 stage install 含 `perception_profiling`，该 pair
作废并保留日志，不用手工改 cache/link evidence。

## 6. Analyzer Comparison Order

`stage_latency_calibration.py` 按以下顺序 fail closed：

1. 三个目录/role/mode/duration/workload/per-run validity；
2. shared source/runner/workload/CPU provenance；
3. new closure manifest presence、schema、artifact SHA 重算；
4. paired source、dependency comparison、helper set 和 closure roots equality；
5. OFF/ON target identity 与 callback/full exact target equality；
6. pidstat scope/bracket/sample count；
7. CPU 与 callback percentile thresholds。

因此旧 mixed-prefix evidence 在读取 CPU 前退出 code 2；threshold exceeded 的完整 paired
evidence 才退出 code 1；通过才退出 0。

## 7. Tests And Operational Gates

### 7.1 Synthetic

- 扩展当前 `make_build_evidence()` 创建两个 dependency build/install trees、三个 helper
  ELF fixture 与 YAML；测试 canonical ordering 后 digest 稳定。
- 逐项 fault injection：missing/duplicate artifact、path escape、wrong package set、Release/
  O3/missing frame pointer、main install literal、header/library/helper/source digest mismatch。
- calibration fixtures默认写 schema-1 closure artifact/fields；删除或变更任一字段必须在
  pidstat/threshold 前失败。原 CPU bidirectional 和 p50/p95/p99 tests 不变。
- static runner assertions 验证 stage 使用 closure setup + target local setup、非 stage
  仍走单 prefix、manifest/copy/checksum 顺序正确。

### 7.2 Short Smoke

新 pair 依次运行 10 秒 unprobed、callback、full 到三个新目录。三者要求：

- target/helper prefix 按角色正确且无主 workspace install；
- observation/revision 约 10 Hz、无 backlog/reject；
- callback/full 0 lost/unmatched/nesting/incomplete；
- `valid=true`、`normal_completion=true`、角色均退出、checksum 全通过；
- source/dependency/helper digests 相同，callback/full target SHA/build ID 相同；
- 不调用 120 秒 aggregator，不把 smoke CPU/latency 写入 baseline。

### 7.3 Formal Hold Point

short smoke 后停止。只有用户明确授权才顺序创建三组 `>=120 s` raw 并运行 schema-2
aggregator。正式失败不得触发阈值修改或 production 优化。

## 8. Compatibility And Rollback

- C1 wrapper/analyzer 不调用 closure capture；若 C1 tests 变化，回滚 shared API 扩张，
  把逻辑留在 local-map helper。
- C2 非 stage mode 保持原单-prefix default。若 split-prefix 影响其他 modes，恢复其旧
  resolution，只保留 stage-only explicit branch。
- 现有 production prefix 无 build-time source identity 时，不降级为 mtime；改用 fresh
  pair root。
- manifest 集合不够时先更新 C2 固定 domain 表与 tests；不得自动发现并接受未知
  workspace package。
- stage overlay 污染时保留 build/log 诊断并换新目录重建；不编辑生成的 cache、link
command 或 raw manifest。

ROS 2 Jazzy 的 generated setup scripts 在 `env -i` 下会读取未定义的
`AMENT_TRACE_SETUP_FILES`；因此仅在 source ROS/production setup 的窄区间关闭 nounset，
完成后立即恢复 `set -u`。其余 build 命令仍保持 `set -e` 与 `pipefail`。
- old raw、旧 aggregate 与 parent checkbox 均不原地修改。schema-2 formal evidence只有
 在完整 paired run 后新增。
