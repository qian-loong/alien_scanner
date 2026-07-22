# Research: 消息认证与信任边界

- Query: 稳定 identity/session/epoch 之外，父架构是否需要定义消息来源认证和重放防护边界？
- Scope: internal
- Date: 2026-07-21

## Findings

- D-015 的 `vehicle_id`、source/session 和 namespace 解决寻址与逻辑身份问题，但本身不能证明消息确实来自该 vehicle 或 coordinator。
- D-014、D-016、D-027 已定义 epoch、sequence、白名单和旧命令拒绝，可防止一部分逻辑重放，但尚未定义凭据、信任根、撤销和伪造消息处理。
- 稀疏 Relay 会转发 role、task、map 和 control envelope；若只信任 payload 中的 ID，Relay 或错误节点可以伪造任务、角色或健康状态。
- 当前 Phase 3 运行在受控本地网络，没有认证/加密实现，也不应因为本轮架构规划而立即选择具体 PKI、DDS-Security 或厂商密钥系统。
- 完整安全基础设施会引入密钥生命周期、设备入网、证书轮换、撤销、存储和运维边界；但父级至少需要定义“未认证消息不得改变控制权威”的逻辑不变量。

## Implication

推荐本轮定义可替换的 trust/auth adapter 和逻辑 envelope 约束：producer/coordinator identity、session、authority epoch、sequence、credential state 和 replay window 必须可验证；认证失败、凭据撤销或来源未知的消息只能进入诊断，不能改变 map、membership、role、task 或 MotionIntent authority。C1-C8 可先使用确定性 trusted fixture 和 spoof/replay/expired-credential 故障注入，不宣称已完成真实网络加密；具体 PKI、DDS-Security 或密钥管理作为部署子任务选择。

## Accepted Direction

父级接受逻辑 trust/auth adapter 和 envelope 拒绝边界；真实 PKI、DDS-Security、密钥轮换与设备凭据运维不在父任务内选择。确定性测试必须证明未认证、伪造、重放、旧 epoch、撤销和过期凭据不会改变控制状态。
## Evidence

- `.trellis/tasks/07-21-perception-swarm-architecture-refactor/research/identity-session-assessment.md:17`
- `.trellis/tasks/07-21-perception-swarm-architecture-refactor/design.md:372`
- `.trellis/tasks/07-21-perception-swarm-architecture-refactor/design.md:417`
- `.trellis/tasks/07-21-perception-swarm-architecture-refactor/design.md:224`
