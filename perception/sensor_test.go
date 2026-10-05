package perception

import (
	"math"
	"math/rand"
	"testing"

	"github.com/huchangjun118/perception-targeting-go/geom"
)

type testRand struct{ r *rand.Rand }

func (t testRand) Float64() float64     { return t.r.Float64() }
func (t testRand) NormFloat64() float64 { return t.r.NormFloat64() }

func newRand(seed int64) testRand { return testRand{rand.New(rand.NewSource(seed))} }

func baseSensor() Sensor {
	return Sensor{Cfg: SensorConfig{
		ID: "s1", FOVDeg: 120, Range: 50, RateHz: 10,
	}}
}

func worldWith(targets ...Target) WorldView {
	return WorldView{Targets: targets}
}

func TestScanFOVAndRange(t *testing.T) {
	s := baseSensor()
	inFront := Target{ID: "t1", Pos: geom.Vec2{X: 10, Y: 0}, Radius: 1}
	behind := Target{ID: "t2", Pos: geom.Vec2{X: -10, Y: 0}, Radius: 1}
	toofar := Target{ID: "t3", Pos: geom.Vec2{X: 100, Y: 0}, Radius: 1}
	w := worldWith(inFront, behind, toofar)
	obs := s.Scan(0, 10, "self", geom.Vec2{}, 0, w, newRand(1))
	if len(obs) != 1 || obs[0].TargetID != "t1" {
		t.Fatalf("expected only t1 observed, got %+v", obs)
	}
	if obs[0].Visibility != Full {
		t.Fatalf("expected full visibility, got %v", obs[0].Visibility)
	}
}

func TestScanOcclusionLevels(t *testing.T) {
	s := baseSensor()
	tgt := Target{ID: "t1", Pos: geom.Vec2{X: 10, Y: 0}, Radius: 2}
	// 完全遮挡：墙盖住整个圆盘
	wallFull := WorldView{
		Obstacles: []geom.Segment{{A: geom.Vec2{X: 8, Y: -5}, B: geom.Vec2{X: 8, Y: 5}}},
		Targets:   []Target{tgt},
	}
	if obs := s.Scan(0, 10, "self", geom.Vec2{}, 0, wallFull, newRand(1)); len(obs) != 0 {
		t.Fatalf("expected fully occluded target unobserved, got %+v", obs)
	}
	// 部分遮挡：墙只挡住中心
	wallPart := WorldView{
		Obstacles: []geom.Segment{{A: geom.Vec2{X: 8, Y: -0.5}, B: geom.Vec2{X: 8, Y: 0.5}}},
		Targets:   []Target{tgt},
	}
	obs := s.Scan(0, 10, "self", geom.Vec2{}, 0, wallPart, newRand(2))
	if len(obs) != 1 || obs[0].Visibility != Partial {
		t.Fatalf("expected partial visibility, got %+v", obs)
	}
	// 动态物体遮挡
	blocker := Target{ID: "b", Pos: geom.Vec2{X: 5, Y: 0}, Radius: 3}
	wDyn := worldWith(tgt, blocker)
	if obs := s.Scan(0, 10, "self", geom.Vec2{}, 0, wDyn, newRand(3)); len(obs) != 1 || obs[0].TargetID != "b" {
		t.Fatalf("expected only blocker observed, got %+v", obs)
	}
}

func TestScanRateAndDrop(t *testing.T) {
	s := baseSensor()
	s.Cfg.RateHz = 5 // 全局 10Hz → 每 2 tick 一次
	tgt := Target{ID: "t1", Pos: geom.Vec2{X: 10, Y: 0}, Radius: 1}
	w := worldWith(tgt)
	rng := newRand(1)
	var due []uint64
	for tick := uint64(0); tick < 10; tick++ {
		if len(s.Scan(tick, 10, "self", geom.Vec2{}, 0, w, rng)) > 0 {
			due = append(due, tick)
		}
	}
	if len(due) != 5 {
		t.Fatalf("expected 5 updates in 10 ticks, got %v", due)
	}
	// 必丢帧
	s.Cfg.RateHz = 10
	s.Cfg.DropProb = 1.0
	if obs := s.Scan(0, 10, "self", geom.Vec2{}, 0, w, newRand(1)); obs != nil {
		t.Fatal("expected drop with prob 1")
	}
}

func TestScanNoiseDeterministic(t *testing.T) {
	s := baseSensor()
	s.Cfg.NoiseStdPos = 0.5
	tgt := Target{ID: "t1", Pos: geom.Vec2{X: 10, Y: 0}, Radius: 1}
	w := worldWith(tgt)
	a := s.Scan(0, 10, "self", geom.Vec2{}, 0, w, newRand(42))
	b := s.Scan(0, 10, "self", geom.Vec2{}, 0, w, newRand(42))
	if len(a) != 1 || len(b) != 1 || a[0].Position != b[0].Position {
		t.Fatalf("same seed must give same noise: %+v vs %+v", a, b)
	}
	if a[0].Position == a[0].TruePos {
		t.Fatal("expected noise to move position")
	}
	if d := a[0].Position.Dist(a[0].TruePos); d > 5 {
		t.Fatalf("noise unreasonably large: %v", d)
	}
}

func TestFuse(t *testing.T) {
	obs := []Observation{
		{SensorID: "s2", TargetID: "t1", Position: geom.Vec2{X: 10, Y: 1}, Visibility: Partial, Weight: 1},
		{SensorID: "s1", TargetID: "t1", Position: geom.Vec2{X: 10, Y: -1}, Visibility: Full, Weight: 1},
		{SensorID: "s1", TargetID: "t2", Position: geom.Vec2{X: 5, Y: 5}, Visibility: Full, Weight: 1},
	}
	snap := Fuse("a1", 7, obs)
	if snap.PerceiverID != "a1" || snap.Tick != 7 || len(snap.Targets) != 2 {
		t.Fatalf("bad snapshot: %+v", snap)
	}
	p, ok := snap.Find("t1")
	if !ok {
		t.Fatal("missing t1")
	}
	if p.Position != (geom.Vec2{X: 10, Y: 0}) {
		t.Fatalf("expected fused position (10,0), got %v", p.Position)
	}
	if p.Visibility != Full {
		t.Fatalf("expected best visibility full, got %v", p.Visibility)
	}
	if len(p.Contributors) != 2 || p.Contributors[0] != "s1" || p.Contributors[1] != "s2" {
		t.Fatalf("contributors not sorted/explainable: %v", p.Contributors)
	}
	// 目标按 ID 排序
	if snap.Targets[0].TargetID != "t1" || snap.Targets[1].TargetID != "t2" {
		t.Fatalf("targets not sorted: %+v", snap.Targets)
	}
}

func TestScanBearingSign(t *testing.T) {
	s := baseSensor()
	s.Cfg.FOVDeg = 360
	up := Target{ID: "t1", Pos: geom.Vec2{X: 0, Y: 10}, Radius: 1}
	obs := s.Scan(0, 10, "self", geom.Vec2{}, 0, worldWith(up), newRand(1))
	if len(obs) != 1 || math.Abs(obs[0].Bearing-math.Pi/2) > 1e-9 {
		t.Fatalf("expected bearing +pi/2, got %+v", obs)
	}
}
