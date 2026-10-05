# perception-targeting-go

面向场景仿真的进程内感知评估与目标选择库。多个感知者（perceiver）在同一场景中
通过各自配置的传感器感知动态实体，系统负责可见性/遮挡判定、传感器融合、感知
记忆、威胁评分、目标选择与互斥的目标声明。整套系统不依赖外部服务，场景与传感
器输入全部由内存对象或本地 JSON 文件提供。

## 快速开始

```go
cfg := perception.WorldConfig{ /* ... */ }
if err := perception.ValidateConfig(&cfg); err != nil { /* 拒绝非法配置 */ }
w := perception.NewWorld(cfg)
w.Run(100)                       // 推进 100 个 tick
for _, snap := range w.History("p1") {
    fmt.Println(snap.Tick, snap.Chosen, snap.Claimed, snap.Scores)
}
```

配置可序列化：`SaveConfig` / `LoadConfig`（JSON，带版本号与自动迁移）。

## 感知语义

### 传感器与融合

- 每个感知者可挂载多个传感器（`SensorConfig`）：视场角 `fov`（弧度，全角）、
  量程 `range`、更新频率 `rateHz`（相对世界 tick 时长换算为整数更新周期）、
  位置噪声 `noisePos`（高斯标准差，米）与丢帧概率 `dropProb`。
- 每个 tick，所有到期传感器的探测结果被合并为一份 `Snapshot`：
  `Detections`（原始逐传感器探测）→ `Fused`（按目标融合，位置以噪声方差倒数
  加权平均，记录贡献传感器列表）→ `Tracks`（记忆）→ `Scores` → `Chosen`/`Claimed`。
  快照的每一环都可解释：任一融合结果都能追溯到具体传感器与可见性分类。

### 可见性与遮挡

- 静态障碍物与动态实体都会遮挡视线，遮挡按**角域覆盖**计算：从观察点看，目标
  张角区间被遮挡物张角区间的并集覆盖的比例即为被遮挡比例。
- 判定结果三档可区分：`FullyVisible`（可见比例 = 1）、`PartiallyVisible`
  （0 < 比例 < 1）、`NotVisible`（= 0，含超量程、出视场）。视场覆盖不足同样
  按比例计入。
- 边界判定是确定的：所有边界比较使用固定 epsilon（1e-9），同一输入永远得到
  同一结果，与求值顺序无关。

### 感知记忆与时间衰减

- 目标离开视野后按 `memory.ttlTicks` 保留记忆轨迹（`Track`），置信度从 1 线性
  衰减；衰减中的轨迹以 `FromMemory=true` 标记。
- 超过 TTL 的轨迹被判定过期并移除，**不再参与评分与决策**。
- 迟到信息（时间戳早于已存轨迹的观测）会被拒绝，不会覆盖较新的状态。

### 威胁评分与目标选择

每条有效轨迹按四项加权评分（`ScoringConfig` 配置权重）：

```
total = ( wDistance·距离项 + wHeading·朝向项 + wVisibility·持续可见项
        + wThreat·威胁项 ) × 记忆置信度
```

- 距离项：`1 - clamp(d / maxDistance)`，越近越高。
- 朝向项：目标相对感知者朝向的夹角余弦映射到 [0,1]，正前方为 1。
- 持续可见项：累计可见 tick 数 / `visNorm`，饱和于 1。
- 威胁项：实体的固有威胁度（[0,1]）。
- 记忆置信度作为乘性因子，使过期边缘的陈旧记忆自然降权。

`ScoreBreakdown` 记录每一项的取值，评分完全可解释。平局裁决是确定的：
总分相同（浮点精确相等）时，目标 ID 字典序较小者胜，与候选顺序无关。

### 目标声明、抢占与释放

`ClaimRegistry` 保证任一目标同一时刻至多一个占用者，规则与到达顺序无关：

1. 未声明或已过期的目标归请求者；
2. 优先级高出当前持有者 `claims.preemptMargin` 以上的请求总是抢占成功；
3. 持有超过 `claims.holdTicks` 后，优先级相近（margin 以内）且 owner ID 字典序
   更小的请求者抢占成功——这是确定性的平局裁决；
4. 其余情况当前持有者保留目标。

声明可由持有者显式 `Release`，或超过 `claims.ttlTicks` 未刷新后过期。

### 并发推进与确定性

- `World.Step` 分两阶段：感知阶段只读共享状态，可用 worker 池并行
  （`workers` 配置）；声明阶段按感知者 ID 顺序串行裁决。
- 每个感知者的随机数发生器由 `(seed, perceiverID)` 确定性派生，噪声与丢帧
  序列与调度无关。
- 每个感知者有独立的更新节奏（由传感器频率决定），慢感知者只占用自己的
  worker，不会阻塞其他感知者的推进。
- **确定性保证**：固定随机种子与相同输入序列下，逐帧快照与目标选择结果逐条
  一致，与 worker 数量、goroutine 调度及遍历顺序无关（有测试覆盖：
  `TestDeterminismAcrossWorkerCounts`、`TestScaleDeterministicAcrossWorkerCounts`）。

## 配置序列化与版本

- `SaveConfig` 写出 `{"version": 2, "world": {...}}`；`LoadConfig` 读取任意受
  支持版本并自动迁移到当前版本。
- v1 → v2 迁移：传感器单一 `noise` 字段拆分为 `noisePos`/`dropProb`
  （dropProb 默认 0，保持旧行为）；记忆 `ttl` → `ttlTicks`；补充默认声明配置
  与评分 `visNorm`。迁移后的配置可复现等价结果（有测试覆盖）。
- 非法或残缺配置被明确拒绝，错误为 `*ValidationError`，`Code` 可区分原因：
  `malformed_data`（JSON 解析失败）、`unsupported_version`、`missing_field`、
  `out_of_range`、`duplicate_id`。

## 主要配置项

| 字段 | 含义 |
| --- | --- |
| `tickDuration` | 每 tick 的时长（秒），决定传感器更新周期换算 |
| `seed` | 全局随机种子 |
| `entities[]` | 动态实体：位置/速度/半径/威胁度/朝向 |
| `obstacles[]` | 静态障碍物：位置/半径 |
| `perceivers[].sensors[]` | `fov` / `range` / `rateHz` / `noisePos` / `dropProb` |
| `perceivers[].memory.ttlTicks` | 记忆保留时长 |
| `perceivers[].scoring` | 评分权重与归一化参数 |
| `claims` | `preemptMargin` / `holdTicks` / `ttlTicks` |
| `workers` | 感知阶段并行度，0/1 为串行 |

## 测试

```
go test ./...        # 全部测试
go test -race ./...  # 并发竞态检测
go test -bench .     # 规模基准（64 感知者 × 128 实体）
```

覆盖：遮挡（静态/动态/部分可见/边界确定性）、噪声与丢帧复现、记忆过期与
迟到拒绝、目标抢占与互斥声明、并发推进与确定性复现、配置序列化与版本迁移、
非法配置拒绝。
