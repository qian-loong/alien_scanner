# C2 stage calibration build equivalence - 实施计划

## 1. 冻结失败合同与测试入口

- [x] 在 `scripts/test_analyze_local_map_profile.py` 中先加入 old mixed-prefix comparison
  负例，要求新 aggregator 在读取 CPU threshold 前因 closure provenance 缺失/不等价失败。
- [x] 冻结现有 C2 28 tests、C1 19 tests、runner CLI 文本和旧 raw per-run readability；
  记录 implementation 开始前结果，不运行任何 workload。
- [x] 明确 schema 版本：closure artifact v1，stage calibration aggregate/quality v2；旧
  aggregate v1 只保留历史，不原地升级。

## 2. 扩展 local-map build provenance

- [x] 在 `scripts/lib/local_map_build_provenance.py` 保留 `validate_build()` 现有调用，新增
  C2 fixed-domain closure capture/canonicalization helper，不抽到 C1 common parser。
- [x] 固定验证 `perception_core`、`perception_interfaces` build cache、非测试 compile
  entries、headers 和 installed libraries；复用现有 required/conflicting flag 逻辑。
- [x] 记录三个 production helper ELF 与 workload YAML，生成
  `target-workspace-closure.json`、dependency comparison digest、helper-set digest。
- [x] 对 duplicate/missing/path escape/broken symlink/unknown package、无 hash/build ID、
  main workspace install 和 source identity mismatch fail closed。
- [x] 扩充 synthetic build fixture 和 canonical reordering、Release/O3、frame-pointer、
  artifact mutation 正反测试。

## 3. Stage-only prefix split

- [x] 在 `scripts/profile-local-map.sh` 增加 stage-only 的 closure install/build/source
  identity 环境输入；现有六参数 CLI 与非 stage resolution 不变。
- [x] stage source production `setup.bash` 后只 source target overlay `local_setup.bash`；
  target package 必须来自 overlay，profiling package/helper/YAML 必须来自 production。
- [x] 在任何 oracle/target/ROS role 启动前扫描环境、CMake cache、compile database、
  `link.txt`、RPATH/`ldd` 和 closure artifacts，拒绝主 workspace install。
- [x] 把 closure JSON 及新增 fields 写入 run manifest，并确保最终 `sha256sum.txt` 覆盖；
  invalid cleanup/normal_completion 语义不变。
- [x] 添加 runner static assertions：stage split 正向、缺 closure inputs、helper 落 overlay、
  dependency 落主 workspace、非 stage 行为回归。

## 4. Calibration comparator

- [x] 在 `scripts/lib/stage_latency_calibration.py` 加载并重算每个 closure artifact，先比较
  source/dependency/helper digests 与 roots，再进入 target/pidstat/threshold 逻辑。
- [x] 强制 unprobed 使用 production OFF target，callback/full 使用同一 stage ON target；
  三组 helper set 与 dependency artifacts 来自同一 production closure。
- [x] aggregate/quality schema 升为 2；结构/provenance invalid 仍由 CLI exit 2，完整 paired
  threshold fail 仍 exit 1，pass exit 0。
- [x] 加入 missing artifact/field、artifact SHA、closure root/digest、source/helper mismatch、
  callback/full target mismatch 负例；原双向 CPU 和三个 percentile 门测试保持原值。

## 5. 静态质量门

- [x] 运行：

  ```bash
  bash -n scripts/profile-local-map.sh
  python3 -m py_compile \
    scripts/lib/local_map_build_provenance.py \
    scripts/lib/stage_latency_calibration.py \
    scripts/analyze-local-map-stage-calibration.py
  python3 -m unittest scripts/test_analyze_local_map_profile.py
  python3 -m unittest scripts/test_analyze_perception_profile.py
  ```

- [x] 对 preserved three-run raw 用临时 output 调用新 analyzer，要求 exit 2 且原目录/
  `/tmp/alien-c2-stage-calibration-20260728.json` SHA 不变。
- [x] 代码审核达到 `0 Blocking / 0 High`；修复仅重跑静态/合成测试，不提前构建或运行。

## 6. Fresh paired build

- [x] 用户批准 implementation 后，选择全新 `ALIEN_C2_PAIR_ROOT`，按 `design.md` 命令先
  生成 source identity，再以 single worker 构建 OFF production 九包 closure。
- [x] `env -i` source ROS/production setup 时仅在该窄区间 `set +u`，随后立即恢复
  `set -u`；不得因 Jazzy setup 读取未定义变量而放弃 `set -e`/`pipefail`。
- [x] 重算 source identity，验证九包 cache/install prefix/flags、closure tests 和无主
  workspace install；失败时保留 root 日志并停止。
- [x] source production underlay，以另一组 base 只构建 ON `perception_local_map` overlay；
  不构建 `perception_profiling` 或 dependencies。
- [x] 再次重算 source identity并直接运行 closure validator；验证 core/interfaces exact
  artifacts 与 helper SHA/build IDs 来自 production，stage provider 只出现在 ON target。
- [x] 保存 pair build command、source identity、cache/compile/link/ldd/nm/build IDs、artifact
  hashes及 pair-level checksum，不进入 Git 大文件。

## 7. Short smoke

- [x] 使用新目录顺序运行 10 秒 production unprobed、stage callback、stage full；stage
  显式传入 production closure install/build/source identity。
- [x] 验证三组 `valid=true`、`normal_completion=true`、10 Hz、无 backlog/reject、trace
  零 lost/unmatched/nesting/incomplete、角色退出和 checksum。
- [x] 机器比较 source/dependency/helper digests 完全相同、callback/full target exact
  identity 相同、unprobed OFF/stage ON，且所有 evidence 无主 workspace install。
- [x] 将 smoke 只记为 plumbing/build gate；不得调用正式 aggregator或引用其性能数字。

## 8. Formal rerun hold point

- [x] 审核修复触及 `scripts/`，canonical source identity 已改变；已新建 **v8** fresh pair
  并重跑三组 10 秒 smoke（见下方 2026-07-29 v8 evidence）。旧 v3 只保留为 review 前历史
  证据；v4/v5 与早期 v6 尝试因 source identity / fixed-domain / gzip 非确定性 / untracked
  收集等问题作废；**v6 经 2026-07-29 独立审计发现三项过程瑕疵后整体降级为历史证据**
  （见下方审计记录）；v7 因驱动脚本 stage 测试块 `PAIR_ROOT` unbound variable 中止作废。
  以上均不得用于当前代码正式重跑。
- [x] 已汇报/记录 code review、paired build 和 smoke evidence；**仍等待用户明确授权**
  正式 120 秒校准。未授权时停止，task 保持 `in_progress`，父任务 calibration checkbox
  不动，且不得调用 schema-2 formal aggregator 或引用 short smoke 性能数字。
- [x] 2026-07-29 获用户授权后（方案 B 推进顺序确认），以 /tmp 内驱动（不触碰身份域）
  创建三个全新 120 秒 output 并严格顺序运行：
  `/tmp/alien-c2-formal-unprobed-20260729-v8`、`/tmp/alien-c2-formal-callback-20260729-v8`、
  `/tmp/alien-c2-formal-full-20260729-v8`；运行前身份复算与 v8 盖章 byte-相同；
  workload、affinity、event set、duration、threshold 均未改变。
- [x] schema-2 aggregator 已运行并保存
  `/tmp/alien-c2-stage-calibration-20260729-v8.json`
  （`b26accdbe07a3a8312304d06636fa6f22cb6af7b9bb237107f2f8eaf45ca2d70`）与 quality
  （`c620e94a7ef4215959a6433b207e9c6b64312dd24d5353ddb2a946fa0673555e`），
  `AGGREGATOR_EXIT=1`：CPU 门通过（full−unprobed delta 0.067 pp，阈值 1.7617 pp；
  三组 mean 35.2343/34.7904/35.1673%），callback p50（59 901 ns）/p95（935 077 ns）
  通过，**仅 p99 失败**（2 815 445 ns > 2 308 828 ns）。有效 raw 全部保留，已创建
  独立 finding `.trellis/tasks/07-29-c2-stage-p99-tail-overhead`，本 task 不做优化。
  CPU 近零差同时证实旧 2.097 pp 超标确为构建不等价所致，本 task 的核心目标
  （build equivalence）达成。

## 9. 收口与回滚

- [x] 更新 child/parent research 与 implement 状态，只写已执行事实；不覆盖旧 evidence。
- [x] 运行 `task.py validate`、JSON parse、`git diff --check`，汇报文件、测试、raw 路径和
  未决风险；未经用户明确授权不提交、不 push、不 archive。
- [ ] 回滚点：C1 回归时撤回 shared 影响；非 stage 回归时恢复单-prefix分支；closure
  capture 过宽时收敛到固定两依赖/三 helper；污染 prefix 只弃用并换新 root，不修生成物。

## Risky Files

- `scripts/lib/local_map_build_provenance.py`: 新 schema/canonical digest 的唯一所有者。
- `scripts/profile-local-map.sh`: stage-only prefix split 和 workload 前 fail-closed 边界。
- `scripts/lib/stage_latency_calibration.py`: valid/invalid/threshold exit 分类。
- `scripts/test_analyze_local_map_profile.py`: synthetic closure 和旧证据拒绝回归。

不应修改 `ws/src/alien_perception/perception_local_map/`、C1 wrapper、公共 ROS 接口或
production defaults。

## 2026-07-28 Paired Build And Short Smoke Evidence

- Paired root：`/tmp/alien-c2-stage-pair-20260728-v3`；image：
  `sha256:6eb20770ab231c3a9e270b63c469fe12356d1d99f51b991f34d6ba65d88f0d52`；
  source revision：`06a5d7fe66f13316fa76eba41daf58f5694509eb`；paired source identity：
  `c9feb80814bf7b5cbfbb0f5a69480da235d5dbd1af7fac53e022382c9fc3ffe9`。
- OFF production 九包 sequential/single-worker build 和 ON local-map-only overlay build
  均完成。隔离环境重跑结果分别为 `266 tests, 0 errors, 0 failures` 与
  `47 tests, 0 errors, 0 failures`；`forbidden-main-install-hits.txt` 为空。
- production/stage closure JSON 字节相同，SHA-256 均为
  `bdca0207bf89fecdc884a8d3564eb05779923c8ee3b15e5e851b4cf847d0f495`；dependency
  comparison 为 `c1c369a937e175c11f0c0317791e2bb813c30f24b798849fad5185a236c2e54e`；
  helper set 为 `14509d238d4f39bfc02a59977f4991d6d136966423724ac558007442edb04b73`。
  live validator 重算 core 11 个、interfaces 98 个 artifacts 和 3 个 helper 均通过。
- Pair evidence checksum manifest 为
  `/tmp/alien-c2-stage-pair-20260728-v3/pair-evidence-sha256.txt`，自身 SHA-256 为
  `788fe13b4113f8787291c7e30de29033cfa0e1b8fcf86d9ab4660f3a39d2669f`，全部条目复核为
  `OK`。
- 三组 10 秒 short smoke 严格顺序完成：
  `/tmp/alien-c2-paired-smoke-unprobed-20260728-v3`、
  `/tmp/alien-c2-paired-smoke-callback-20260728-v3`、
  `/tmp/alien-c2-paired-smoke-full-20260728-v3`。三者均为 `valid=true`、
  `normal_completion=true`，约 10 Hz 且无 backlog/reject；callback/full 的 lost、
  unmatched、nesting、incomplete、duplicate、invalid duration 和 unexpected-event
  counters 全部为 0。
- 三份 raw checksum manifest SHA-256 依次为
  `d7803ce5e00908237f325071df4bb7c7c9c1904096afbdf0020b803f82baa354`、
  `dbda22206a1b0035bbd2b195e3e3ecc93868616ca2b3f9ad14f56415f4ab7710`、
  `3e680ec5cb212565a132a23f1827d37fe0289d267073b474d137d1dd933ac592`；所有记录 PID
  均不存在，LTTng 无残留 session。
- 本次只关闭 short plumbing/build gate；未调用 schema-2 formal aggregator，未运行三组
  120 秒正式校准，也未把 short smoke latency/CPU 数字写成 baseline。原三份 formal raw
  与旧 aggregate SHA-256 再次核对未变化。
- review 前回归为 C2 `33 tests`、C1 `19 tests` 全通过；`bash -n`、`py_compile`、
  `task.py validate`、JSON/JSONL parse 和 `git diff --check` 均通过。


## 2026-07-28 v6 Fresh Pair And Short Smoke Evidence

- 当前有效 paired root：`/tmp/alien-c2-stage-pair-20260728-v6`；image：
  `sha256:6eb20770ab231c3a9e270b63c469fe12356d1d99f51b991f34d6ba65d88f0d52`；
  source revision：`06a5d7fe66f13316fa76eba41daf58f5694509eb`；
  `paired_source_identity_sha256=f617c3e25e0def515dd5da2357a0c6efc6d6fb69a6e11c933e11853d5645a7ee`。
  source identity 算法与 runner 对齐：`SOURCE_INPUT_PATHS=(.devcontainer scripts ws/src
  CMakeLists.txt)` + `git diff --binary` + `git ls-files --others` +
  `create_deterministic_source_archive`（`tar --sort=name --mtime=@0 ... | gzip -n`）。
- production OFF 九包 sequential/`gmake -j1` 与 stage ON 仅 `perception_local_map`
  overlay 均完成。隔离环境测试：production rerun `266 tests, 0 errors, 0 failures,
  0 skipped`；stage `47 tests, 0/0/0`；`forbidden-main-install-hits.txt` 为空。
- production/stage closure JSON 字节相同：
  `workspace_closure_manifest_sha256=ffed8fd553d8d6cd9c41e1c6ad0bbe8cf686955d5d15f46f3d83ed0bf6da0ddf`；
  `workspace_dependency_comparison_sha256=ab8cd3b60a8096b71efa43ad14a868906288f4847836d1ba0e936a60ea997615`；
  `profiling_helper_set_sha256=7e46181ee2f35490d211cbf446e9151783cdc7226b38e2eb0a1d672ffcf0ef96`。
  pair evidence manifest：`/tmp/alien-c2-stage-pair-20260728-v6/pair-evidence-sha256.txt`
  （自身 SHA-256 `90efc1934d859f7a3196d9f34c062d4a8f7f1de106674bd02ac7faa09e197ec8`）。
- 实现修复：`scripts/lib/local_map_build_provenance.py` 只审计 production local-map
  compile entries，避免 `TestCaveFullRayScene` / `TestLocalMap` 等 test 目标把
  `cave_world` / `drone_scanner` / `perception_fixtures` 误判为 production 依赖泄漏；
  `scripts/test_analyze_local_map_profile.py` 增加“test entry 应忽略 / production 引用
  fixtures 必须失败”覆盖。容器内 C2+C1 unittest 曾报 55 OK。
- 三组 10 秒 short smoke 严格顺序完成，且 `cross_smoke_digest_ok`：
  - unprobed：`/tmp/alien-c2-paired-smoke-unprobed-20260728-v6`；`plain-sample`；
    stage **OFF**；`target_sha256=c3c858a350d62ffd6289b00948a0a492571228563a02a960d3611be3e690112b`；
    `valid=true` / `normal_completion=true`；`sha256sum.txt`
    `67d0e1fa0001f67558f0d7d961e7becc678304de9526caac50655542768a5305`。
  - callback：`/tmp/alien-c2-paired-smoke-callback-20260728-v6`；`stage-latency` /
    event_set=callback；stage **ON**；
    `target_sha256=da7996b45d4304cab948af210827e860ddd3261d202b854bb7b8a06742a5208c`；
    `valid=true` / `normal_completion=true`；`sha256sum.txt`
    `0f8e9d6ffeeb65d175ce8f9f4020277b89de107803dc0e00ff2a9e8d95671c5a`。
  - full：`/tmp/alien-c2-paired-smoke-full-20260728-v6`；`stage-latency` /
    event_set=full；同一 stage target；`valid=true` / `normal_completion=true`；
    `sha256sum.txt` `5dad2afb248d6ea8861fa203d17dea6fdb04c242228f92f8c0818303684f92ca`。
  三组 source/dependency/helper digests 与 pair 记录一致；callback/full target exact
  identity 相同；unprobed 使用 production OFF target。日志尾：
  `/tmp/alien-c2-v6-final-smokes.log` → `FINAL_SMOKES_COMPLETE` +
  `HOLD: formal 120s calibration not run; awaiting user authorization`。
- 作废历史（仅保留失败原因，不可复用）：
  - v3：review 改 `scripts/` 后 source identity 失效；
  - v4：test 目标误伤 fixed-domain；
  - v5/早期 v6：gzip mtime 非确定性、错误 untracked 收集、smoke env / build-base 配错。
- 本次只关闭 short plumbing/build gate；**未**调用 schema-2 formal aggregator，**未**
  运行三组 120 秒正式校准，也未把 short smoke latency/CPU 数字写成 baseline。task 保持
  `in_progress`；父任务 stage calibration checkbox 保持未勾选。

## 2026-07-28 Independent Code Review

- 审核修复了 paired source/run raw 未绑定、dependency build path 可越界、workload SHA
  未绑定 closure、固定域外 workspace dependency 未拒绝、aggregate stale pass output 和
  stage 参数失败遗留空目录；schema-2 CLI 的 exit `0/1/2` 均有合成覆盖。
- 修复后 `bash -n`、`py_compile`、C2 `36 tests` 与 C1 `19 tests` 全部通过；preserved
  mixed-build raw 仍 exit 2，三份 raw checksum 和旧 aggregate SHA-256
  `40e9d7a9444d8a9a57b98649d0ac018183b07865bfcbae8d4e5ba9a0185884ff` 未变化。代码审核为
  `0 Blocking / 0 High`。
- v3 production/stage closure 仍可由 live validator 重算为相同 SHA-256
  `bdca0207bf89fecdc884a8d3564eb05779923c8ee3b15e5e851b4cf847d0f495`，pair evidence
  checksum 也仍通过；但本轮修复改变了 `scripts/`，因此 v3 paired source identity 不再
  等于当前源码身份。必须 fresh build v4 并重跑三组短 smoke 后，才可请求正式三组授权。
- 上述 “必须 fresh build v4” 的 hold 已被后续 **v6** pair + 三组 10 秒 smoke 关闭；
  v3 仍只作历史证据。正式 120 秒校准与 schema-2 aggregator 仍未运行、未授权；未提交或
  push；task 保持 `in_progress`，父任务 stage calibration checkbox 保持开放。

## 2026-07-29 v6 独立审计与 v8 重做记录

- 对产生 v6 的 Codex 会话（149 个工具调用）做了全量轨迹审计，发现三项过程瑕疵：
  1. **首次 production 测试实际失败过**（`drone_scanner` `test_fake_odom_integration`
     1 failure，16:25），rerun 通过后 `cp -f` 覆盖了 `test-result-production.txt`，
     implement.md 未披露首次失败；原始证据仍在 v6 `log-test-production/` 与 continue.log。
  2. **source identity 为构建完成近 6 小时后重盖**（二进制 16:04，盖章 22:02），期间在
     身份域 `scripts/` 下新增 6 个 driver 脚本；逐文件比对确认 scope 内 tracked diff 未
     变、新增文件非构建输入，二进制实际未受影响，但与 v3 判废标准双重标准。
  3. **身份算法会话中切换两次**，before-build 与最终身份非同一算法，"构建时源码 ==
     f617..." 只能间接推出。
- 结论：v6 数字实际可信但过程不合格，降级为历史证据；采用方案 B 重做。
- **v7**（root `/tmp/alien-c2-stage-pair-20260729-v7`）：production 九包构建 + 首轮测试
  一次通过（266/0/0），但驱动脚本 stage 测试隔离块误用 `${PAIR_ROOT}`（应为
  `${ALIEN_C2_PAIR_ROOT}`）触发 unbound variable 中止；按标准修脚本即身份域变更，v7
  整体作废，不重盖、不续跑。

## 2026-07-29 v8 Fresh Pair And Short Smoke Evidence（当前有效）

- 驱动：`scripts/run-c2-stage-pair-v8.sh`（单脚本一次成型；运行期间身份域零改动；每阶段
  后重算 identity 并 `cmp`，漂移即失败，不允许重盖；测试首轮结果永不覆盖，失败时单次
  rerun 写独立文件并打 `FIRST_RUN_FAILED` 标记——本次未触发，两侧均首轮通过）。
- paired root：`/tmp/alien-c2-stage-pair-20260729-v8`；image：
  `sha256:6eb20770ab231c3a9e270b63c469fe12356d1d99f51b991f34d6ba65d88f0d52`（launch 时
  `docker inspect` 实取，经 `ALIEN_PAIR_IMAGE_ID` 注入，非硬编码）；source revision：
  `06a5d7fe66f13316fa76eba41daf58f5694509eb`；
  `paired_source_identity_sha256=53e54c67ad8e206dc6fa93cb34a6b33963df4daf09ddbe1696c249ac2c7dff6f`。
  身份从一开始即用 runner 对齐算法（`SOURCE_INPUT_PATHS=(.devcontainer scripts ws/src
  CMakeLists.txt)` + `git diff --binary` + `git ls-files --others` +
  `create_deterministic_source_archive`），预构建双算稳定性检查通过；production build /
  production tests / stage build / stage tests / smokes 五个检查点身份全部 byte-相同。
- production OFF 九包 sequential/`-j1` 构建 9min11s；隔离测试**首轮**
  `266 tests, 0 errors, 0 failures, 0 skipped`（无 rerun）。stage ON 仅
  `perception_local_map` overlay 构建 1min34s；隔离测试首轮 `47 tests, 0/0/0`。
  `forbidden-main-install-hits.txt` 为空。
- production/stage closure JSON 字节相同：
  `workspace_closure_manifest_sha256=8599630723dce6ad6878d8bc6d4091733dcdfdd864ffc7d2d00755024711b9e2`；
  `workspace_dependency_comparison_sha256=569ee83cbcc333574670209f7d1282074435eead5fd13f9fb22a9472e3fbe87c`；
  `profiling_helper_set_sha256=d4459545bebf824d4daa21d6b26a3664d4b733f0ccf848b99ac6892f256b95e7`。
  pair evidence manifest：`/tmp/alien-c2-stage-pair-20260729-v8/pair-evidence-sha256.txt`
  （自身 SHA-256 `17174cf2834ff6832b7337693b5e46d185186f10b8b50e9ab302b51dcc3472d7`，
  含 `test-result-production-first.txt` 与 `test-result-stage.txt`）。
- 三组 10 秒 short smoke 严格顺序完成，全部 `smoke_gate_ok` 且 `cross_smoke_digest_ok`：
  - unprobed：`/tmp/alien-c2-paired-smoke-unprobed-20260729-v8`；`plain-sample`；stage
    **OFF**；`target_sha256=bda41357f10610440fc9c5e9c53e03c8694066c9d5f5e7f564f9288637186bc1`；
    `sha256sum.txt` `d001e7bc1d8d23f923bc6c943a9041c1267c62fd22772fa3a7bdfafdc70487f0`。
  - callback：`/tmp/alien-c2-paired-smoke-callback-20260729-v8`；`stage-latency`/callback；
    stage **ON**；`target_sha256=9308ee2b53f946d28abcfe95b5216fea1613306f283d31d5092254925e041821`；
    `sha256sum.txt` `9a141c7fabe8c0a0de70d84a8b833f4cc791e11d4c7ffb0636ed0faabaa3a8ba`。
  - full：`/tmp/alien-c2-paired-smoke-full-20260729-v8`；`stage-latency`/full；同一 stage
    target（`target_sha256`/`target_build_id` 与 callback 相同）；`sha256sum.txt`
    `154407fc4033405208007c84a0e706d899477f034afa8f0f7ac6344469c9204c`。
  三组 source/dependency/helper digests 与 pair 记录一致；unprobed 用 production OFF
  target。`lttng list` 无残留 session，无残留 target 进程。日志：
  `/tmp/alien-c2-stage-pair-20260729-v8-driver.log` → `V8_PAIR_COMPLETE` +
  `HOLD: formal 120s calibration not run; awaiting user authorization`。
- 本次只关闭 short plumbing/build gate；**未**调用 schema-2 formal aggregator，**未**运行
  三组 120 秒正式校准，也未把 short smoke latency/CPU 数字写成 baseline。task 保持
  `in_progress`；父任务 stage calibration checkbox 保持未勾选。

---

## 收口（2026-07-31）：目标已达成

本 finding 要解决的是"production 与 stage 依赖构建不等价，导致校准不可采信"。
该目标已达成：

- 补齐了传递依赖 provenance 与拒绝门（`workspace closure` 门），并由 41 个合成
  正反例覆盖伪造场景。
- 交付了配对构建 v8 与 v11，两侧隔离测试首轮通过（production `266/0/0/0`、
  stage `47/0/0/0`），closure/helper digests 跨 production/stage/smoke 一致。
- 该门在 v10 上**真实拦截过一次**：分析侧判据代码在 pair 盖章后才修改，
  stage-latency 300 s 被正确拒绝——这是"分析侧代码也属身份域"这条教训的来源。

遗留的未勾选项不再执行：父任务已按受限交付收口，后续无配对测量需求。

**后续注意**：git 会把若干 CRLF 文件规范化为 LF，那会改变工作区内容、作废
v11 pair 身份。若将来要复现配对测量，必须重建 pair。
