# 第三阶段：策略、外部 producer 与 Recorder Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在已完成的双进程 FakeBackend 链路上交付可验证的 ONNX Locomotion、外部 producer 接入，以及可追溯、可按时间读取的 Recorder。

**Architecture:** application 监督线程独占行为 FSM、授权与请求；本地推理线程和外部 producer 只生成候选首帧及后续目标。runtime 继续独占执行权和安全边界。Recorder 从 runtime 同一控制 tick 的相干快照经有界队列异步落盘，读取工具只读文件。

**Tech Stack:** C++17、CycloneDDS/cyclonedds-cxx、nlohmann/json、ONNX Runtime C++、Python 3/onnx/numpy、CMake/CTest、FakeBackend。

**Spec:** `docs/superpowers/specs/references/refactor-report.md` §3.1–3.4、§4、§6.2–6.3、§8、§10–11.1；前两阶段基线见 `docs/superpowers/plans/2026-09-25-application-foundation.md` 和 `docs/testing.md`。

## Global Constraints

- 本阶段只在 FakeBackend 上验收。真机数值、RealBackend、驱动、零点及台架验收属于第四阶段；不得把示例配置称为上机安全配置。
- `application` 与 `robot_runtime` 保持独立进程；runtime 不链接 ONNX，外部 producer 不取得监督会话、模式请求、Reset 或心跳发送权。
- 每条 Tracking `JointCommand` 包含全部关节的 q/dq/tau_ff/kp/kd；单位为 rad、rad/s、N·m、N·m/rad、N·m·s/rad。运行边界只由 runtime 配置执行。
- ONNX 模型级唯一键 `irmv.deployment` 是 UTF-8 JSON，`schema_version=1`。模型最终 SHA-256 在导出后计算并写日志，不自嵌入模型。
- 策略频率由元数据指定；报告中的这次训练为 50 Hz、动作 scale=0.5、12 轴预期观测 45 维，实际仍以实例化环境与模型 I/O 验证。PPO `clip_param=0.2` 不作动作裁剪。
- 运动中不得热换模型或 producer；停到 Damping 后重新 Initializing → Ready → Locomotion。任何停止、急停、故障、超时或重启均不得让旧 epoch 恢复。
- DDS、磁盘写入和推理不得阻塞 runtime 控制 tick；队列有界且丢弃/缺口可观测。
- 第一方 C++ 使用仓库固定的 Google clang-format 18.1.8；依赖固定版本和来源，不以相邻旧 SDK 的私有构建物代替。
- 修改相邻训练仓时遵守其 `AGENTS.md`：使用 `env_humanoid`、Ruff/pre-commit；训练仓改动和 SDK 改动分别提交。无法重建匹配环境时明确报告模型交付未通过，禁止填猜测映射。

## Review Focus

1. 缺失、重复、未知版本或非有限 ONNX 元数据：启动前拒绝模型，绝不沿用旧数组。对应 Task 1、3。
2. 动作与两个观测关节列表顺序不同，或训练名含 `_joint`：按显式名字分别映射并验证完全覆盖。对应 Task 1、3。
3. 推理迟到、失败、首帧超期以及已停止后的结果：不能发布旧 epoch 命令，必须触发上层故障/停止。对应 Task 4。
4. 旧准备回复、producer 重启、授权通知丢失以及意图过期：不能产生授权或延长运动。对应 Task 5、6。
5. Recorder 队列满、进程意外退出、文件尾损坏或读取跨越缺口：控制持续运行，读取明确报告可用数据和缺口。对应 Task 7、8。

---

## 文件与交付边界

| 工作单元 | 文件职责 |
|---|---|
| 元数据契约 | `schema/onnx-deployment-v1.schema.json` 固定 JSON 字段；`include/irmv/policy/deployment.hpp`、`src/policy/deployment.cpp` 解析/校验并建立三组按名映射；`tests/unit/deployment_test.cpp` 固定合法及拒绝样例。 |
| 训练导出 | 相邻 `irmv_humanoid_locomotion/scripts/rsl_rl/play.py` 在现有 ONNX 导出流程中写元数据；该仓新增专用纯数据提取/校验模块及测试。SDK `tools/verify_policy.py` 检查模型与前后推理等价。 |
| 本地策略 | `include/irmv/policy/{onnx_model,observation,action,inference}.hpp` 与对应 `src/policy/` 文件分别负责模型加载、观测、动作、ORT 工作线程；`src/application/{behavior_fsm,application_loop}.cpp` 负责准入和退出。 |
| 部署配置 | `config/sdk.schema.json`、`include/irmv/configuration/application_config.hpp`、`src/configuration/application_config.cpp` 增加模型路径、可选按模型摘要匹配的覆盖及期限；不改变 runtime 配置摘要语义。 |
| 外部协议 | `protocol/producer.idl`、`include/irmv/protocol/producer.hpp`、`src/transport/producer_transport.cpp` 固定生命周期与 DDS 端点；`src/application/producer_supervisor.cpp` 只管理准备/授权通知。 |
| 记录与读取 | `include/irmv/telemetry/{record,recorder,record_reader}.hpp`、`src/telemetry/` 对应实现、`tools/record_to_npz.py`、`tools/read_record.cpp`。 |
| 验收 | 各自单元测试、`tests/integration/` 中跨进程场景、`docs/testing.md` 和 `README.md` 的运行与追溯说明。 |

> 现有 `protocol/execution.idl` 的五类执行消息、`RobotClient` 的确认语义、runtime FSM 与安全守卫是基线。若 recorder 需要更多同 tick 数据，先在 runtime 内定义快照并补单元测试；不要把记录字段塞进高频 `RobotState` 或让 DDS 回调写磁盘。

### Task 1: 冻结 ONNX 部署 JSON 与 SDK 校验器

**Files:** Create `schema/onnx-deployment-v1.schema.json`, `include/irmv/policy/deployment.hpp`, `src/policy/deployment.cpp`, `tests/unit/deployment_test.cpp`; modify `CMakeLists.txt`.

**Interfaces:** `DeploymentContract ParseDeployment(std::string_view json)`；`JointMapping BuildJointMapping(const DeploymentContract&, const std::vector<std::string>& robot_joint_names)`，分别保存 action、joint_pos_rel、joint_vel_rel 的索引。`DeploymentContract` 包含 model I/O 名称和维度、`policy_period_us`、有序关节表、默认 q/dq、kp/kd、动作 scale/offset/两层 clip/顺序、观测 term 描述、normalizer 与来源。所有不支持的 term、坐标系、处理顺序显式拒绝。

- [ ] **Step 1: 写 RED。** 合法 2 轴夹具令 action、位置观测、速度观测三个顺序各不相同；断言三个映射分别正确，所有数组长度与契约声明的模型 I/O 匹配。逐一断言缺键、版本 2、NaN/Inf、缺关节、重复名、不支持的 transform/normalizer、未解释的空值被拒绝并带字段路径；模型级重复 key 留给 Task 3 的加载测试。
- [ ] **Step 2: 运行 RED。** `cmake --build build --target deployment_test && ctest --test-dir build -R '^deployment_test$' --output-on-failure`；预期新目标或断言失败。
- [ ] **Step 3: 实现最小解析与 schema。** JSON root 固定 `schema_version=1`、`policy_type="joint_position"`、`model`、`source`、`action`、`observation`、`normalizer`；`action` 中显式声明各数组对应有序关节，观测每个 term 固定 name/dimension/unit/frame/scale/clip/history。模型级 key 重复由读取层检查。所有浮点递归检查有限性；`null` 仅在明定可空的 clip 和来源缺失说明中允许。
- [ ] **Step 4: 运行 GREEN 并提交。** 同一 CTest PASS；`cmake --build build --target format-check` PASS；提交契约、解析器和测试。这里不读取训练 YAML，也不导出模型。

### Task 2: 从匹配训练环境导出完整模型包

**Files:** Modify `../irmv_humanoid_locomotion/scripts/rsl_rl/play.py`; create该仓 `scripts/rsl_rl/deployment_metadata.py`、`scripts/rsl_rl/test_deployment_metadata.py`; create SDK `tools/verify_policy.py`、`tests/integration/policy_package_test.py`; modify `CMakeLists.txt`。

**Interfaces:** 导出模块接收已经实例化的 action/observation manager、资产、checkpoint 与训练配置路径，返回 Task 1 JSON；`verify_policy.py <before.onnx> <after.onnx>` 检查元数据、ONNX checker、ORT 读回与固定输入输出等价。

- [ ] **Step 1: 写 RED。** 用实例化环境或录下的解析后描述符测试三份有序关节名单、默认 q/dq、解析后 offset、逐关节 kp/kd、观测 term 顺序/维度、normalizer 处理位置；缺少解析后的名单必须失败。用已有无元数据 ONNX 测试补写只替换单个 key，保留其它 metadata，图/权重/固定输入结果不变。
- [ ] **Step 2: 运行 RED。** 在训练仓 `conda activate env_humanoid` 后运行 `python scripts/rsl_rl/test_deployment_metadata.py`；SDK 运行 `python3 tests/integration/policy_package_test.py`；预期失败。
- [ ] **Step 3: 在原导出入口实现。** 从与 checkpoint 对应的实例化环境取动作/观测解析值；本次训练的 100/6、60/4、50 Hz、scale 0.5 与两个 `null` clip 用作交叉检查，不能充作完整数组来源。记 checkpoint/配置摘要、代码/资产版本或明确的缺失说明。离线补写工具仅作已验证输入的迁移路径，使用安全纯数据 YAML 解析且拒绝未知 Python 标签/函数，不从原始正则猜关节顺序。
- [ ] **Step 4: GREEN 与证据。** `onnx.checker.check_model` PASS；ORT 读回唯一 key；同一输入逐项输出等价。提供实际生成模型的 SHA-256、来源、12 轴映射覆盖和训练端对照样本路径；若缺匹配环境/资产，提交导出器与测试，并把真实模型验收明确列为未通过门槛，不虚报完成。分别提交两个仓库的改动。

### Task 3: 应用部署配置、ORT 加载与观测/动作纯函数

**Files:** Modify `config/sdk.schema.json`, `config/sdk.json`, `include/irmv/configuration/application_config.hpp`, `src/configuration/application_config.cpp`, `cmake/Dependencies.cmake`, `CMakeLists.txt`; create `include/irmv/policy/{onnx_model,observation,action}.hpp`, `src/policy/{onnx_model,observation,action}.cpp`, `tests/unit/{policy_config,policy_math,onnx_model}_test.cpp`.

**Interfaces:** `PolicyConfig` 持有可选模型路径和按完整模型 SHA-256 匹配的 `policy_overrides`；本仓 Fake 默认配置的路径为空，仅选择 local 来源时要求非空。`OnnxModel::Load(path, robot_joint_names) -> OnnxModel` 返回已校验的契约、映射、实际 I/O 与摘要；`BuildObservation(const DeploymentContract&, const JointMapping&, const protocol::RobotState&, const application::VelocityIntent&, const std::vector<float>& last_action) -> std::vector<float>`；`BuildCommand(const DeploymentContract&, const JointMapping&, const std::vector<float>& action, const protocol::RobotState&, const PolicyConfig&) -> protocol::JointCommand`。

- [ ] **Step 1: 写 RED。** 测试配置覆盖仅在 SHA-256 与关节名同时匹配时生效，并记录原值/新值；不改变 `runtime.config_id`。用三种不同关节顺序与非零默认位姿，断言观测六段顺序 `command → projected_gravity → base_ang_vel → joint_pos_rel → joint_vel_rel → last_action`、维度和坐标系，动作先按元数据规定的裁剪/缩放/offset 顺序产生 q，kp/kd 跟 action 名对应。无 clip 不裁剪；异常/缺失反馈、维度不符、非有限结果、模型级重复 key 和真实 I/O 不一致均拒绝。
- [ ] **Step 2: 运行 RED。** `cmake --build build --target policy_config_test policy_math_test onnx_model_test && ctest --test-dir build -R '^(policy_config|policy_math|onnx_model)_test$' --output-on-failure`。
- [ ] **Step 3: 实现。** 固定 ONNX Runtime 版本、来源和库校验，CMake 只给 `irmv_policy`/`application` 链接；加载时用 `GetModelMetadata().LookupCustomMetadataMapAllocated("irmv.deployment", allocator)`，校验实际输入输出名称/形状。按名字合并增益覆盖并在上层预检位置/目标速度/预计力矩及切换连续性；不修改 ONNX，不让模型边界覆盖 runtime safety。
- [ ] **Step 4: GREEN 与边界检查。** 单元测试 PASS；`ldd build/robot_runtime` 无 ONNX Runtime；无模型时 `robot_runtime` 仍能启动 FakeBackend。提交配置、依赖、纯函数与测试。

### Task 4: 本地推理线程与 Locomotion 行为接入

**Files:** Create `include/irmv/policy/inference.hpp`, `src/policy/inference.cpp`, `tests/unit/inference_test.cpp`; modify `include/irmv/application/{behavior_fsm,input_router,application_loop}.hpp`, `src/application/{behavior_fsm,input_router,application_loop,main}.cpp`, `config/{sdk.json,sdk.schema.json}`, `CMakeLists.txt`; add integration scenario `tests/integration/scenarios/application_local_policy.json`.

**Interfaces:** `InferenceWorker::Submit(PolicyInput)` 接受 `{generation, epoch, state_seq, state_stamp_us, intent_seq, observation}` 的最新值；`TryTake(PolicyOutput*)` 返回同一输入身份及动作或错误。将 `InputEvent::StartLocomotion` 追加在现有枚举末尾，默认键 `l`；`BehaviorFsm::OfferLocomotionCandidate(protocol::JointCommand, protocol::TimeUs)` 保存待请求首帧，`Step(..., StartLocomotion, ...)` 仅在 Ready、模型/反馈/意图有效且候选仍新鲜时提出 Tracking 请求；`BehaviorStep` 继续承载要发送的完整命令。CLI `--locomotion-source local|external` 在启动时选择本次运行来源，默认 local；运动中不热换。

- [ ] **Step 1: 写 RED。** 假推理引擎可控地返回即时、迟到、异常、非有限结果；断言 Ready 预热不发新源目标，首帧与新 epoch 原子请求，Applied 加新鲜状态后才到 Locomotion；重复启动不重置 last_action/历史；停止、急停、Fault、runtime 重启时旧结果不发包；首帧/运行超时锁存上层故障并请求停止。
- [ ] **Step 2: 运行 RED。** `cmake --build build --target inference_test application_fsm_test && ctest --test-dir build -R '^(inference|application_fsm)_test$' --output-on-failure`。
- [ ] **Step 3: 实现。** 单工作线程与有界最新值通道；进入新授权时初始化 last_action/历史，退出时递增本地 generation 并清空候选。监督循环每 tick 先处理状态/急停/退出，再取新结果和普通输入；速度意图过期归零。结果必须匹配 generation、epoch、源状态并在期限内；推理与 DDS 回调不直接推进 FSM。
- [ ] **Step 4: GREEN 与跨进程验收。** 上述测试、`integration_application_local_policy` PASS；注入慢推理时 runtime 500 Hz tick 不被阻塞，目标到期自行回退。记录本地模型摘要、配置标识、实际命令及输出动作；提交。

### Task 5: 冻结外部 producer 生命周期协议和传输

**Files:** Create `protocol/producer.idl`, `include/irmv/protocol/producer.hpp`, `include/irmv/transport/producer_transport.hpp`, `src/transport/producer_transport.cpp`, `tests/integration/producer_codec_test.cpp`; modify `CMakeLists.txt`, `docs/runtime-protocol.md`.

**Interfaces:** 独立的 `ProducerAnnouncement`、`PrepareRequest`、`PrepareResult`、`AuthorizationNotice`、`StopNotice`、`VelocityIntentMessage`。每条消息含 `protocol_version=1`、16 字节 producer 进程实例身份；准备相关消息含非零 `prepare_id`，授权/停止通知关联该编号。准备/通知另含 runtime 实例、监督会话、producer_id、epoch。速度意图含 body frame 的 forward/lateral m/s、yaw rad/s、递增 seq、主机发送时间与有效期；意图到期归零。topic 固定前缀 `irmv/v1/producer/` 加六个消息的 snake_case 名；生命周期 QoS 为 reliable/volatile/keep-last 8/最大阻塞 10,000 μs，意图为 best-effort/volatile/keep-last 1。执行协议五类消息与 topic 不变。

- [ ] **Step 1: 写 RED。** 编解码与真实 DDS 端点测试：有界身份/数组、版本错、零编号、重复/乱序准备回复、旧实例、过期意图、消息缺字段拒绝；断言生命周期 topic 为可靠有界非持久，意图为有界最新值，发布不会在控制线程发生。
- [ ] **Step 2: 运行 RED。** `cmake --build build --target producer_codec_test && ctest --test-dir build -R '^producer_codec_test$' --output-on-failure`。
- [ ] **Step 3: 实现并文档化冻结字段。** 关联键固定 `{producer_process_id, prepare_id, runtime_id, session_id}`；准备结果只允许 Ready 或 Failed，Ready 携完整首帧；授权通知的 epoch 与 Tracking 请求一致；StopNotice 只通知，不作为 runtime 撤权依据。用现有 16 字节 `Id` 和 `Sequence` 类型，所有 TTL 用微秒。新增 `application.external_producer.prepare_timeout_us=1000000` 和 `registration_timeout_us=1000000` 的 Fake 测试值，速度意图期限沿用现有 `input.intent_timeout_us=500000`；准备期间 application 仍按既有周期发监督心跳。为准备/授权通知/意图配置有界队列及丢弃计数。
- [ ] **Step 4: GREEN，提交。** 编解码和 QoS 测试 PASS；`docs/runtime-protocol.md` 写明默认可信 DDS 网络边界，标识字段不构成发送者认证。

### Task 6: 外部 producer 监督、示例进程与真实 DDS 场景

**Files:** Create `include/irmv/application/producer_supervisor.hpp`, `src/application/producer_supervisor.cpp`, `examples/external_producer.cpp`, `tests/unit/producer_supervisor_test.cpp`; modify application FSM/loop、CMake；add `tests/integration/scenarios/application_external_*.json` 和驱动测试。

**Interfaces:** `ProducerSupervisor::Select(...)` 只在 Ready 接受兼容注册；`Prepare(..., prepare_id, intent)` 产生准备请求；`Observe(PrepareResult, RobotState)` 返回经新鲜度/布局/频率/首帧校验的候选；`Authorized(runtime_id, session_id, producer_id, epoch)` 仅在 RobotClient 确认后发通知；`Revoke(reason)` 清除准备和候选并发停止通知。示例进程只订阅状态/意图、发布完整目标和生命周期回复。

- [ ] **Step 1: 写 RED。** 单元和独立进程 DDS 场景覆盖注册 → 预热 → 原子首帧 → 确认后连续输出；旧准备回复、双 producer、通知丢失、producer 重启、目标暂停/延迟、意图过期、停止/急停后继续发包、supervisor 暂停/杀死。断言只有一个 producer/epoch 实际跟踪，旧包不喂狗，监督超时仍撤会话。
- [ ] **Step 2: 运行 RED。** `ctest --test-dir build -R '^(producer_supervisor_test|integration_application_external_)' --output-on-failure`。
- [ ] **Step 3: 实现。** application 选择来源，runtime 授权仍通过现有 `RobotClient::BeginRequest(Tracking, ..., producer_id, &first_command)`；外部进程无模式/会话/心跳写端点。准备期目标不发布至执行 topic；确认后 seq 从首帧之后严格递增。禁止在 Locomotion 中换源；需先 Damping、重新初始化和 Ready。
- [ ] **Step 4: GREEN，提交。** 跨进程测试 PASS；示例不依赖 ONNX；对未授权、旧 session/epoch/seq、过期 source_state、NaN/Inf 和目标跳变保留现有 runtime 拒绝/Fault 语义。

### Task 7: 相干记录快照与有界 Recorder

**Files:** Create `include/irmv/telemetry/{record,recorder}.hpp`, `src/telemetry/recorder.cpp`, `tests/unit/recorder_test.cpp`; modify `src/runtime/{execution_fsm,runtime_loop}.cpp`, `include/irmv/runtime/` 相应接口、`config/sdk.schema.json`, `config/sdk.json`, `include/irmv/configuration/config.hpp`, `src/configuration/config.cpp`, `CMakeLists.txt`.

**Interfaces:** `RecordSample` 含 `schema_version=1`、runtime/有效配置摘要、布局与单位、`RobotState.stamp_us`/`state_seq`、同 tick 接收和采用的命令身份及完整目标/增益、FSM/故障、最终 `output_action`、有效性及主机发送结果。application 的另一个 `PolicySessionRecord` 事件以 `{runtime_id, session_id, producer_id, epoch}` 关联模型摘要及覆盖原值/新值；外部来源记录进程实例及收到的增益。`Recorder::TryRecord(const RecordSample&) noexcept` 只做有界复制；worker 列式落盘。`RecordGap` 明确 `{first_seq,last_seq,reason}`。配置 `telemetry.recorder={enabled, sample_hz, queue_capacity, directory}`，Fake 默认 100 Hz，可设 500 Hz。

- [ ] **Step 1: 写 RED。** 500 Hz tick 每 5 tick 采 100 Hz，起始相位固定；同一行的 state/command/output 使用同 tick 数据，采样时间取 `RobotState.stamp_us`，保留 `seq`。队列满/慢磁盘时 tick 不等待，缺口计数和范围可读；停止正常 drain，异常截断有可辨识尾部；运行 10 分钟的 FakeBackend 记录检查行数、频率、序号缺口和有界内存。
- [ ] **Step 2: 运行 RED。** `cmake --build build --target recorder_test && ctest --test-dir build -R '^recorder_test$' --output-on-failure`。
- [ ] **Step 3: 实现。** 控制线程在提交输出、生成 RobotState 的同一次 tick 构造一个不可变快照；预分配队列，worker 采用版本化列式分块格式与单独索引/元数据文件，分块有长度及校验，未完成尾块可识别。记录器故障只报告/计数，不在控制线程抛异常或阻塞。日志中的“发送成功”仅表示主机路径接受。
- [ ] **Step 4: GREEN，提交。** 测试 PASS；慢消费者场景确认 runtime 500 Hz 与安全超时照常执行。会话事件通过独立有界记录通道关联，模型摘要缺失时显式写 unknown，不给无模型的 runtime 编造值。

### Task 8: 时间范围读取、NPZ 转换与只读 replay 边界

**Files:** Create `include/irmv/telemetry/record_reader.hpp`, `src/telemetry/record_reader.cpp`, `tools/read_record.cpp`, `tools/record_to_npz.py`, `tests/unit/record_reader_test.cpp`, `tests/integration/record_export_test.py`; modify `CMakeLists.txt`, `README.md`.

**Interfaces:** `RecordReader::ReadRange(TimeUs begin_us, TimeUs end_us, RecordVisitor)` 返回同一运行目录内 `[begin_us,end_us)` 的有序样本和显式缺口；`read_record --from-us --to-us` 输出只读 JSON 行；`record_to_npz.py <run-dir> <out.npz>` 保留列名、单位、joint_names、schema_version、时间/seq 与 gap 表。

- [ ] **Step 1: 写 RED。** 查询空范围、边界恰等、跨 chunk、跨缺口、损坏尾块、不同运行目录和版本；断言返回有序 seq/stamp，不把缺口填成连续数据。NPZ 回读逐列与原记录一致；工具不得打开 DDS 写端点或产生 JointCommand。
- [ ] **Step 2: 运行 RED。** `cmake --build build --target record_reader_test && ctest --test-dir build -R '^(record_reader_test|record_export_test)$' --output-on-failure`。
- [ ] **Step 3: 实现读取和转换。** 使用 Task 7 索引按时间定位分块，逐块校验；未知 schema 清楚拒绝，末尾不完整块返回已完成数据及截断标记。NPZ 仅离线生成；replay 数据接口只读，不实现真机指令回放或旧 dashboard/TUI。
- [ ] **Step 4: GREEN，提交。** 上述测试 PASS；运行真实 10 分钟 FakeBackend 记录，统计频率、丢样和范围检索结果并保存摘要。

### Task 9: 第三阶段端到端验收与交接

**Files:** Modify `docs/testing.md`, `README.md`, `CMakeLists.txt` 的测试登记/安装规则；必要时新增 `tests/integration/phase3_acceptance_test.py`。

**Interfaces:** 安装后 `application` 能以显式模型路径启动本地模式，或选择示例外部 producer；`robot_runtime` 仍只读执行配置并可无模型启动。交接报告列出模型 SHA-256、配置摘要、测试命令与结果、未满足的模型/训练环境门槛。

- [ ] **Step 1: 先写验收脚本。** 真实独立进程串联 `Standby → Damping → Initializing → Ready → Locomotion → Damping → Standby`，逐 tick 验证 50 Hz 目标/500 Hz 控制的状态与输出；在本地和外部两模式分别测急停、超时、重启、旧授权、迟到 Applied。训练端和 SDK 对同一状态/意图的观测向量、模型动作、目标位置逐项比较。
- [ ] **Step 2: 执行全量验证。** `cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug && cmake --build build -j2 && ctest --test-dir build --output-on-failure && cmake --build build --target format-check`；有具体并发疑点时加定向 TSan，核心安全路径加现有 ASan/UBSan 构建。
- [ ] **Step 3: 安装树验证。** 用临时 prefix `cmake --install build --prefix <tmp-prefix>` 后在脱离源码路径的工作目录启动两个进程，检查 ONNX Runtime 运行库、模型路径、相对 rpath、配置和只读记录工具。不得把测试模型或相邻训练仓绝对路径写成生产默认值。
- [ ] **Step 4: 文档与提交。** `docs/testing.md` 给出每项 §11/§11.1 的测试证据、10 分钟 Recorder 及 30 分钟状态切换的内存/线程/句柄统计、剩余第四阶段台架项。只有训练模型包和对照数据真实通过时才标为 ONNX 一致性完成；分别提交测试、文档与安装修正。

## 执行顺序与交接

按 Task 1 → 2 → 3 → 4 → 5 → 6 → 7 → 8 → 9 的依赖顺序推进。Task 5 的协议设计可在 Task 2 的训练环境重建期间准备，但交接评审仍逐任务过 RED→GREEN→审查→提交；共享工作树避免并发修改 CMake、配置及 FSM。每个任务交给实现 agent 时同时提供本文、总体报告和该任务的前置提交。Task 2 实际模型包若因匹配训练环境不可用而卡住，可继续完成外部 producer 与 Recorder，但 Task 9 不得将本地 ONNX 链路宣称验收通过。
