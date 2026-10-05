package sim

import (
	"fmt"
	"reflect"
	"testing"

	"github.com/huchangjun118/perception-targeting-go/geom"
	"github.com/huchangjun118/perception-targeting-go/perception"
	"github.com/huchangjun118/perception-targeting-go/targeting"
)

// testScene 构造确定性测试场景：两个代理、两个移动目标、一堵墙。
func testScene() Scene {
	mkSensor := func(id string) perception.SensorConfig {
		return perception.SensorConfig{
			ID: id, FOVDeg: 360, Range: 60, RateHz: 10,
			NoiseStdPos: 0.2, DropProb: 0.1,
		}
	}
	w := targeting.ScoreWeights{Distance: 1, Heading: 0.5, Persist: 0.5, Threat: 1, Confidence: 0.5}
	mem := perception.MemoryConfig{TTLTicks: 5, Decay: 0.9}
	return Scene{
		StepHz: 10,
		Obstacles: []geom.Segment{
			{A: geom.Vec2{X: 15, Y: -5}, B: geom.Vec2{X: 15, Y: 5}},
		},
		Targets: []perception.Target{
			{ID: "t1", Pos: geom.Vec2{X: 20, Y: 10}, Radius: 1, Threat: 0.9,
				Velocity: geom.Vec2{X: -0.5, Y: 0}},
			{ID: "t2", Pos: geom.Vec2{X: 20, Y: -10}, Radius: 1, Threat: 0.4,
				Velocity: geom.Vec2{X: -0.3, Y: 0.1}},
		},
		Agents: []Agent{
			{ID: "a1", Pos: geom.Vec2{}, Sensors: []perception.SensorConfig{mkSensor("s1"), mkSensor("s2")},
				Weights: w, MemCfg: mem, MaxRange: 100},
			{ID: "a2", Pos: geom.Vec2{X: 0, Y: 5}, Sensors: []perception.SensorConfig{mkSensor("s1")},
				Weights: w, MemCfg: mem, MaxRange: 100},
		},
	}
}

func run(t *testing.T, workers, frames int) []FrameResult {
	t.Helper()
	e := NewEngine(testScene(), 42, workers)
	var out []FrameResult
	for i := 0; i < frames; i++ {
		out = append(out, e.Step())
	}
	return out
}

func TestDeterminismSameSeed(t *testing.T) {
	a := run(t, 1, 30)
	b := run(t, 1, 30)
	if !reflect.DeepEqual(a, b) {
		t.Fatal("same seed must reproduce identical frames")
	}
}

func TestDeterminismAcrossWorkerCounts(t *testing.T) {
	serial := run(t, 1, 30)
	parallel := run(t, 8, 30)
	if !reflect.DeepEqual(serial, parallel) {
		for i := range serial {
			if !reflect.DeepEqual(serial[i], parallel[i]) {
				t.Fatalf("frame %d differs between worker counts:\n%+v\n%+v", i, serial[i], parallel[i])
			}
		}
	}
}

func TestNoDoubleClaim(t *testing.T) {
	frames := run(t, 4, 30)
	for _, f := range frames {
		seen := make(map[string]string)
		for target, agent := range f.Claims {
			if prev, dup := seen[target]; dup {
				t.Fatalf("tick %d: target %s claimed by both %s and %s", f.Tick, target, prev, agent)
			}
			seen[target] = agent
		}
	}
}

func TestClaimsEventuallyMade(t *testing.T) {
	frames := run(t, 2, 20)
	// 目标持续靠近，若干帧后应有声明产生保全场景活性
	any := false
	for _, f := range frames {
		if len(f.Claims) > 0 {
			any = true
		}
	}
	if !any {
		t.Fatal("expected at least one claim over 20 frames")
	}
}

func TestOcclusionAffectsSnapshots(t *testing.T) {
	// t1 在墙后（y=10 方向无墙，t1 实际可见）；构造专门场景验证墙后目标不可见
	scene := Scene{
		StepHz: 10,
		Obstacles: []geom.Segment{
			{A: geom.Vec2{X: 10, Y: -20}, B: geom.Vec2{X: 10, Y: 20}},
		},
		Targets: []perception.Target{
			{ID: "hidden", Pos: geom.Vec2{X: 20, Y: 0}, Radius: 1, Threat: 0.5},
		},
		Agents: []Agent{{
			ID: "a1", Pos: geom.Vec2{},
			Sensors:  []perception.SensorConfig{{ID: "s1", FOVDeg: 360, Range: 100, RateHz: 10}},
			Weights:  targeting.ScoreWeights{Threat: 1},
			MemCfg:   perception.MemoryConfig{TTLTicks: 3, Decay: 0.9},
			MaxRange: 100,
		}},
	}
	e := NewEngine(scene, 1, 1)
	f := e.Step()
	if len(f.Snapshots[0].Targets) != 0 {
		t.Fatalf("target behind wall must be invisible, got %+v", f.Snapshots[0].Targets)
	}
	if f.Decisions[0].TargetID != "" {
		t.Fatalf("no target should be selected, got %q", f.Decisions[0].TargetID)
	}
}

func TestPreemption(t *testing.T) {
	// 两个代理看同一目标，b 的评分更高（距离更近），应抢占 a 的声明。
	w := targeting.ScoreWeights{Distance: 1}
	mem := perception.MemoryConfig{TTLTicks: 5, Decay: 0.9}
	mkAgent := func(id string, x float64) Agent {
		return Agent{
			ID: id, Pos: geom.Vec2{X: x},
			Sensors: []perception.SensorConfig{{ID: "s1", FOVDeg: 360, Range: 200, RateHz: 10}},
			Weights: w, MemCfg: mem, MaxRange: 200,
		}
	}
	scene := Scene{
		StepHz: 10,
		Targets: []perception.Target{
			{ID: "t1", Pos: geom.Vec2{X: 100, Y: 0}, Radius: 1, Threat: 0.5},
		},
		Agents: []Agent{mkAgent("a-far", 0), mkAgent("b-near", 90)},
	}
	e := NewEngine(scene, 7, 1)
	f := e.Step()
	// 同帧内 a-far 先声明（ID 序），b-near 分更高应抢占
	if f.Claims["t1"] != "b-near" {
		t.Fatalf("expected b-near to hold claim, got %v", f.Claims)
	}
}

func TestSlowAgentDoesNotBlockOthers(t *testing.T) {
	// 大规模场景：多代理多目标，验证推进完成且结果确定。
	scene := testScene()
	for i := 0; i < 30; i++ {
		scene.Agents = append(scene.Agents, Agent{
			ID:  fmt.Sprintf("extra-%02d", i),
			Pos: geom.Vec2{X: float64(i), Y: float64(-i)},
			Sensors: []perception.SensorConfig{
				{ID: "s1", FOVDeg: 360, Range: 80, RateHz: 10, NoiseStdPos: 0.1},
			},
			Weights:  targeting.ScoreWeights{Distance: 1, Threat: 1},
			MemCfg:   perception.MemoryConfig{TTLTicks: 5, Decay: 0.9},
			MaxRange: 100,
		})
	}
	runWith := func(workers int) []FrameResult {
		e := NewEngine(scene, 99, workers)
		var out []FrameResult
		for i := 0; i < 20; i++ {
			out = append(out, e.Step())
		}
		return out
	}
	a, b := runWith(1), runWith(16)
	if !reflect.DeepEqual(a, b) {
		t.Fatal("large scene results must not depend on worker count")
	}
}
