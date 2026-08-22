# 真机抓取误差排查文档包

本目录只服务于一个问题：系统可以正常完成抓取流程，但机械臂真机末端相对真实目标长期存在约 2–3 cm 的位置误差。

目标是先通过分层、可重复的测试确定误差来源，再决定是否修改手眼标定、TCP/末端几何、抓取候选、深度尺度或执行映射。禁止在没有测量证据时直接修改运行代码。

## 文档索引

- [真机相关文件清单与架构](./01-real-grasp-scope-and-architecture.md)
- [2–3 cm 误差排查测试计划](./02-2cm-error-investigation-plan.md)
- [Claude Code 受限读取与执行提示词](./03-claude-code-prompt.md)
- [当前日志后的最小补测工作清单](./04-follow-up-work-list.md)
- [补测报告模板](./follow-up-report.md)
- [Claude Code 当前轮次最小提示词](./05-claude-code-follow-up-prompt.md)
- [基于 URDF 权威 TCP 长度的新任务计划](./06-urdf-tcp-authoritative-plan.md)
- [MoveIt 运行时一致性与末端误差复测计划](./07-moveit-runtime-consistency-plan.md)
- [夹爪姿态与插入方向规划修改任务](./08-grasp-orientation-planning-modification.md)
- [最小误差假设区分实验计划](./09-minimal-error-hypothesis-experiment-plan.md)
- [当前相机内参权威下的误差验证任务](./10-authoritative-calibration-error-validation-plan.md)
- [Calibration 内参与安全抓取规划联合修改任务](./11-calibration-and-safe-grasp-planning.md)
- [2026-08-20 手眼 TF 与 GraspNet→Piper TCP 朝向审计日志](../2026-08-20-session-summary.md)

## 推荐执行顺序

1. 先阅读 `01-real-grasp-scope-and-architecture.md`，只建立真机主链路上下文。
2. 按 `02-2cm-error-investigation-plan.md` 执行只读检查、仿真/干跑和真机测量。
3. 将测量数据填入测试计划中的记录表。
4. 只有当某个假设被测试数据支持时，才允许修改相应代码。
5. Claude Code 使用 `03-claude-code-prompt.md`，不得自行扫描项目全仓库。
6. 当前已有 T0–T5 日志后，下一轮优先使用 `04-follow-up-work-list.md`，不要重复执行完整排查计划。
7. 用户已裁决 URDF TCP 长度为真实值，TCP 后续工作使用 `06-urdf-tcp-authoritative-plan.md`，不再等待现场物理测量。
8. 当前下一阶段只执行 `07-moveit-runtime-consistency-plan.md`：先验证 MoveIt 请求帧、运行版本、关节状态和笛卡尔 API，再决定是否进行容差 A/B；不得与相机、TCP 或手眼修改混合。
9. 夹爪姿态、插入方向和 approach 路径修改单独执行 `08-grasp-orientation-planning-modification.md`，必须在 07 完成后进行。
10. 为区分固定手眼偏差、随机 MoveIt 偏差和姿态问题，优先执行 `09-minimal-error-hypothesis-experiment-plan.md`；本计划默认不修改代码。
11. 当前已确认 `camera_data/calibration.yaml` 是本项目相机内参权威文件；后续使用 `10-authoritative-calibration-error-validation-plan.md`，不再重复调查手眼标定 K 的来源。
12. 用户已批准执行 `11-calibration-and-safe-grasp-planning.md`；源码实现和离线验证已完成，真机 plan-only/approach-only/完整抓取仍需单独批准。

## 最终需要返回的材料

Claude Code 完成后，至少应返回：

- `measurement-log.md`
- `root-cause-report.md`
- 每个测试的命令、退出码和关键日志
- 输入文件 hash 与测试编号的对应关系
- command / FK / 实测位置的毫米级误差表
- 已排除假设和证据
- 尚未确定时的最小下一步采集项

没有这些原始数据，不能据此决定修改 TCP、手眼标定、感知或 MoveIt 代码。

## 当前已知事实

- 抓取流程能够正常启动、规划和执行。
- 末端相对真实目标存在约 20–30 mm 的稳定位置误差。
- 当前问题尚未证明是单一 TCP 偏移问题。
- 当前相机内参权威文件为 `camera_data/calibration.yaml`；RealSense `CameraInfo`/`meta.mat` 只作为运行时链路一致性检查对象。
- 当前计划默认“先诊断，后修复”，不预设修复数值。
