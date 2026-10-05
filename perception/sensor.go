package perception

import (
	"math"
	"sort"

	"github.com/huchangjun118/perception-targeting-go/geom"
)

// Target 场景中可被感知的目标（动态物体）。
type Target struct {
	ID       string    `json:"id"`
	Pos      geom.Vec2 `json:"pos"`
	Radius   float64   `json:"radius"`
	Velocity geom.Vec2 `json:"velocity"`
	Threat   float64   `json:"threat"` // 固有威胁度 [0,1]
}

// WorldView 感知所需的世界只读视图。
type WorldView struct {
	Obstacles []geom.Segment // 静态障碍边
	Targets   []Target       // 动态物体（也作为遮挡体）
}

// Sensor 实例化传感器。
type Sensor struct {
	Cfg SensorConfig
}

// RandSource 确定性随机源接口（由调用方注入，保证可复现）。
type RandSource interface {
	Float64() float64 // [0,1)
	NormFloat64() float64
}

// due 判定传感器在 tick 是否更新。
// 规则确定：floor(tick*rate/stepHz) 相对上一 tick 增加即更新；tick 0 必更新。
func (s *Sensor) due(tick uint64, stepHz float64) bool {
	if tick == 0 {
		return true
	}
	if s.Cfg.RateHz <= 0 || stepHz <= 0 {
		return false
	}
	cur := math.Floor(float64(tick) * s.Cfg.RateHz / stepHz)
	prev := math.Floor(float64(tick-1) * s.Cfg.RateHz / stepHz)
	return cur > prev
}

// classify 计算目标圆盘的可见性等级：中心与左右边缘三条视线采样。
// 全部遮挡 → Invisible；部分遮挡 → Partial；无遮挡 → Full。
// 采样点固定，判定确定。
func classify(selfPos geom.Vec2, t Target, occluders []geom.Segment, circles []geom.Circle) Visibility {
	to := t.Pos.Sub(selfPos)
	dist := to.Len()
	if dist <= 0 {
		return Full
	}
	dir := to.Normalize()
	perp := geom.Vec2{X: -dir.Y, Y: dir.X}
	// 边缘采样点向内收缩 1e-6，保证边界判定确定（切线不算遮挡）。
	inset := t.Radius - 1e-6
	if inset < 0 {
		inset = 0
	}
	points := [3]geom.Vec2{
		t.Pos,
		t.Pos.Add(perp.Scale(inset)),
		t.Pos.Add(perp.Scale(-inset)),
	}
	visible := 0
	for _, p := range points {
		if !geom.Occluded(selfPos, p, occluders, circles) {
			visible++
		}
	}
	switch visible {
	case 0:
		return Invisible
	case 3:
		return Full
	default:
		return Partial
	}
}

// Scan 在 tick 执行一次扫描。返回本 tick 产生的观测列表（可能因频率/丢帧为空）。
// rng 的调用顺序固定（按目标列表顺序），保证确定性。
func (s *Sensor) Scan(tick uint64, stepHz float64, selfID string, selfPos geom.Vec2,
	selfHeading float64, world WorldView, rng RandSource) []Observation {
	if !s.due(tick, stepHz) {
		return nil
	}
	// 丢帧判定：每次扫描至多消耗一个随机数，顺序固定。
	if s.Cfg.DropProb > 0 && rng.Float64() < s.Cfg.DropProb {
		return nil
	}
	heading := selfHeading + s.Cfg.MountOffset
	var out []Observation
	for _, t := range world.Targets {
		if t.ID == selfID {
			continue
		}
		to := t.Pos.Sub(selfPos)
		dist := to.Len()
		if dist > s.Cfg.Range {
			continue
		}
		bearing := geom.AngleDiff(to.Angle(), heading)
		if s.Cfg.FOVDeg < 360 && math.Abs(bearing) > s.Cfg.FOVDeg*math.Pi/360 {
			continue
		}
		// 遮挡体：静态障碍 + 除自身与目标外的动态物体。
		var circles []geom.Circle
		for _, o := range world.Targets {
			if o.ID == selfID || o.ID == t.ID {
				continue
			}
			circles = append(circles, geom.Circle{Center: o.Pos, Radius: o.Radius})
		}
		vis := classify(selfPos, t, world.Obstacles, circles)
		if vis == Invisible {
			continue
		}
		// 位置噪声：确定性随机源，按目标顺序消耗。
		noisy := t.Pos
		if s.Cfg.NoiseStdPos > 0 {
			noisy.X += rng.NormFloat64() * s.Cfg.NoiseStdPos
			noisy.Y += rng.NormFloat64() * s.Cfg.NoiseStdPos
		}
		out = append(out, Observation{
			SensorID:   s.Cfg.ID,
			TargetID:   t.ID,
			Position:   noisy,
			TruePos:    t.Pos,
			Visibility: vis,
			Range:      dist,
			Bearing:    bearing,
			Weight:     1 / (1 + s.Cfg.NoiseStdPos),
			Tick:       tick,
		})
	}
	return out
}

// Fuse 将同一 tick 多个传感器的观测合并为快照。
// 位置按权重加权平均；可见性取最高等级；贡献者列表排序，保证可解释与确定。
func Fuse(perceiverID string, tick uint64, obs []Observation) Snapshot {
	snap := Snapshot{PerceiverID: perceiverID, Tick: tick}
	byTarget := make(map[string][]Observation)
	var ids []string
	for _, o := range obs {
		if _, ok := byTarget[o.TargetID]; !ok {
			ids = append(ids, o.TargetID)
		}
		byTarget[o.TargetID] = append(byTarget[o.TargetID], o)
	}
	sort.Strings(ids)
	for _, id := range ids {
		list := byTarget[id]
		var wsum, x, y float64
		best := Invisible
		var contrib []string
		for _, o := range list {
			wsum += o.Weight
			x += o.Position.X * o.Weight
			y += o.Position.Y * o.Weight
			if o.Visibility > best {
				best = o.Visibility
			}
			contrib = append(contrib, o.SensorID)
		}
		sort.Strings(contrib)
		conf := float64(len(list)) / 3 // 多传感器冗余提升置信度
		if conf > 1 {
			conf = 1
		}
		conf *= 0.5 + 0.25*float64(best) // 可见性等级加成
		snap.Targets = append(snap.Targets, TargetPercept{
			TargetID:     id,
			Position:     geom.Vec2{X: x / wsum, Y: y / wsum},
			Visibility:   best,
			Contributors: contrib,
			LastSeenTick: tick,
			Confidence:   conf,
		})
	}
	return snap
}
