// Package config 提供场景与感知配置的序列化、版本迁移与校验。
package config

import (
	"encoding/json"
	"fmt"

	"github.com/huchangjun118/perception-targeting-go/sim"
)

// 支持的配置版本范围。v1 缺少 v2 引入的 DropProb/Decay/MaxRange 字段，
// 迁移时填充确定默认值，保证旧数据升级后复现等价结果。
const (
	MinVersion     = 1
	CurrentVersion = 2
)

// ErrKind 配置错误类别（可区分）。
type ErrKind string

const (
	ErrMalformed   ErrKind = "malformed"   // JSON 无法解析
	ErrUnsupported ErrKind = "unsupported" // 版本不受支持
	ErrValidation  ErrKind = "validation"  // 字段非法或残缺
)

// Error 带类别的配置错误。
type Error struct {
	Kind   ErrKind
	Detail string
}

func (e *Error) Error() string { return string(e.Kind) + ": " + e.Detail }

// Envelope 版本化配置信封。
type Envelope struct {
	Version int       `json:"version"`
	Scene   sim.Scene `json:"scene"`
}

// Marshal 将场景写为当前版本的 JSON。
func Marshal(scene sim.Scene) ([]byte, error) {
	if err := Validate(scene); err != nil {
		return nil, err
	}
	return json.Marshal(Envelope{Version: CurrentVersion, Scene: scene})
}

// Unmarshal 读取任意受支持版本的 JSON，迁移到当前版本并校验。
func Unmarshal(data []byte) (sim.Scene, error) {
	var env Envelope
	if err := json.Unmarshal(data, &env); err != nil {
		return sim.Scene{}, &Error{ErrMalformed, err.Error()}
	}
	if env.Version < MinVersion || env.Version > CurrentVersion {
		return sim.Scene{}, &Error{ErrUnsupported,
			fmt.Sprintf("version %d not in [%d,%d]", env.Version, MinVersion, CurrentVersion)}
	}
	if env.Version < CurrentVersion {
		env.Scene = migrate(env.Scene, env.Version)
	}
	if err := Validate(env.Scene); err != nil {
		return sim.Scene{}, err
	}
	return env.Scene, nil
}

// migrate 逐版本升级场景配置，填充新增字段的确定默认值。
func migrate(s sim.Scene, from int) sim.Scene {
	if from <= 1 {
		// v1→v2：补充 v2 新增字段的默认值。
		for i := range s.Agents {
			if s.Agents[i].MaxRange == 0 {
				s.Agents[i].MaxRange = 100
			}
			if s.Agents[i].MemCfg.Decay == 0 {
				s.Agents[i].MemCfg.Decay = 0.95
			}
		}
	}
	return s
}

// Validate 校验场景配置，返回第一个发现的错误。
func Validate(s sim.Scene) error {
	fail := func(format string, args ...interface{}) error {
		return &Error{ErrValidation, fmt.Sprintf(format, args...)}
	}
	if s.StepHz <= 0 {
		return fail("step_hz must be > 0, got %v", s.StepHz)
	}
	agentIDs := make(map[string]bool)
	for _, a := range s.Agents {
		if a.ID == "" {
			return fail("agent with empty id")
		}
		if agentIDs[a.ID] {
			return fail("duplicate agent id %q", a.ID)
		}
		agentIDs[a.ID] = true
		if a.MaxRange <= 0 {
			return fail("agent %q: max_range must be > 0", a.ID)
		}
		if len(a.Sensors) == 0 {
			return fail("agent %q: at least one sensor required", a.ID)
		}
		if a.MemCfg.TTLTicks == 0 {
			return fail("agent %q: memory ttl_ticks must be > 0", a.ID)
		}
		if a.MemCfg.Decay <= 0 || a.MemCfg.Decay > 1 {
			return fail("agent %q: memory decay must be in (0,1], got %v", a.ID, a.MemCfg.Decay)
		}
		sensorIDs := make(map[string]bool)
		for _, sc := range a.Sensors {
			if sc.ID == "" {
				return fail("agent %q: sensor with empty id", a.ID)
			}
			if sensorIDs[sc.ID] {
				return fail("agent %q: duplicate sensor id %q", a.ID, sc.ID)
			}
			sensorIDs[sc.ID] = true
			if sc.FOVDeg <= 0 || sc.FOVDeg > 360 {
				return fail("agent %q sensor %q: fov_deg must be in (0,360], got %v", a.ID, sc.ID, sc.FOVDeg)
			}
			if sc.Range <= 0 {
				return fail("agent %q sensor %q: range must be > 0", a.ID, sc.ID)
			}
			if sc.RateHz <= 0 {
				return fail("agent %q sensor %q: rate_hz must be > 0", a.ID, sc.ID)
			}
			if sc.NoiseStdPos < 0 {
				return fail("agent %q sensor %q: noise_std_pos must be >= 0", a.ID, sc.ID)
			}
			if sc.DropProb < 0 || sc.DropProb > 1 {
				return fail("agent %q sensor %q: drop_prob must be in [0,1], got %v", a.ID, sc.ID, sc.DropProb)
			}
		}
	}
	targetIDs := make(map[string]bool)
	for _, t := range s.Targets {
		if t.ID == "" {
			return fail("target with empty id")
		}
		if targetIDs[t.ID] {
			return fail("duplicate target id %q", t.ID)
		}
		targetIDs[t.ID] = true
		if t.Radius <= 0 {
			return fail("target %q: radius must be > 0", t.ID)
		}
		if t.Threat < 0 || t.Threat > 1 {
			return fail("target %q: threat must be in [0,1], got %v", t.ID, t.Threat)
		}
	}
	for i, o := range s.Obstacles {
		if o.A == o.B {
			return fail("obstacle %d: degenerate segment (endpoints equal)", i)
		}
	}
	return nil
}
