// Package perception 提供传感器建模、可见性融合与感知记忆。
package perception

import "github.com/huchangjun118/perception-targeting-go/geom"

// Visibility 可见性等级。
type Visibility int

const (
	// Invisible 完全不可见（被遮挡或超出视场/量程）。
	Invisible Visibility = iota
	// Partial 部分可见（目标圆盘只有部分视线可达）。
	Partial
	// Full 完全可见。
	Full
)

func (v Visibility) String() string {
	switch v {
	case Invisible:
		return "invisible"
	case Partial:
		return "partial"
	case Full:
		return "full"
	}
	return "unknown"
}

// SensorConfig 单个传感器的配置。
type SensorConfig struct {
	ID          string  `json:"id"`
	FOVDeg      float64 `json:"fov_deg"`       // 视场角（度），>=360 表示全向
	Range       float64 `json:"range"`         // 量程（米）
	RateHz      float64 `json:"rate_hz"`       // 更新频率
	NoiseStdPos float64 `json:"noise_std_pos"` // 位置噪声标准差（米）
	DropProb    float64 `json:"drop_prob"`     // 单次扫描丢帧概率 [0,1]
	MountOffset float64 `json:"mount_offset"`  // 相对感知者朝向的安装偏角（弧度）
}

// Observation 单个传感器对单个目标的一次观测。
type Observation struct {
	SensorID   string
	TargetID   string
	Position   geom.Vec2 // 加噪后的位置
	TruePos    geom.Vec2 // 真实位置（用于解释/调试）
	Visibility Visibility
	Range      float64
	Bearing    float64 // 相对感知者朝向的方位角
	Weight     float64 // 融合权重（由传感器噪声决定）
	Tick       uint64
}

// TargetPercept 融合后的单目标感知结果。
type TargetPercept struct {
	TargetID     string
	Position     geom.Vec2 // 多传感器加权融合位置
	Visibility   Visibility
	Contributors []string // 贡献传感器 ID（有序，可解释）
	LastSeenTick uint64
	Confidence   float64 // [0,1]
}

// Snapshot 一帧的感知快照：某感知者在某 tick 的融合结果。
type Snapshot struct {
	PerceiverID string
	Tick        uint64
	Targets     []TargetPercept // 按 TargetID 排序，保证确定性
}

// Find 按目标 ID 查找感知结果。
func (s *Snapshot) Find(targetID string) (TargetPercept, bool) {
	for _, t := range s.Targets {
		if t.TargetID == targetID {
			return t, true
		}
	}
	return TargetPercept{}, false
}
