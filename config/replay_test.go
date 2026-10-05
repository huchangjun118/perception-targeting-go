package config

import (
	"reflect"
	"testing"

	"github.com/huchangjun118/perception-targeting-go/geom"
	"github.com/huchangjun118/perception-targeting-go/perception"
	"github.com/huchangjun118/perception-targeting-go/sim"
	"github.com/huchangjun118/perception-targeting-go/targeting"
)

func replayScene() sim.Scene {
	w := targeting.ScoreWeights{Distance: 1, Heading: 0.5, Persist: 0.5, Threat: 1, Confidence: 0.5}
	mem := perception.MemoryConfig{TTLTicks: 5, Decay: 0.9}
	mkSensor := func(id string) perception.SensorConfig {
		return perception.SensorConfig{ID: id, FOVDeg: 360, Range: 60, RateHz: 10,
			NoiseStdPos: 0.2, DropProb: 0.1}
	}
	return sim.Scene{
		StepHz: 10,
		Obstacles: []geom.Segment{
			{A: geom.Vec2{X: 15, Y: -5}, B: geom.Vec2{X: 15, Y: 5}},
		},
		Targets: []perception.Target{
			{ID: "t1", Pos: geom.Vec2{X: 20, Y: 10}, Radius: 1, Threat: 0.9,
				Velocity: geom.Vec2{X: -0.5}},
			{ID: "t2", Pos: geom.Vec2{X: 20, Y: -10}, Radius: 1, Threat: 0.4,
				Velocity: geom.Vec2{X: -0.3, Y: 0.1}},
		},
		Agents: []sim.Agent{
			{ID: "a1", Sensors: []perception.SensorConfig{mkSensor("s1"), mkSensor("s2")},
				Weights: w, MemCfg: mem, MaxRange: 100},
			{ID: "a2", Pos: geom.Vec2{Y: 5}, Sensors: []perception.SensorConfig{mkSensor("s1")},
				Weights: w, MemCfg: mem, MaxRange: 100},
		},
	}
}

// TestSerializedReplayEquivalent 序列化→反序列化后的场景推进结果与原场景逐帧一致。
func TestSerializedReplayEquivalent(t *testing.T) {
	scene := replayScene()
	data, err := Marshal(scene)
	if err != nil {
		t.Fatalf("marshal: %v", err)
	}
	restored, err := Unmarshal(data)
	if err != nil {
		t.Fatalf("unmarshal: %v", err)
	}
	run := func(s sim.Scene) []sim.FrameResult {
		e := sim.NewEngine(s, 42, 4)
		var out []sim.FrameResult
		for i := 0; i < 20; i++ {
			out = append(out, e.Step())
		}
		return out
	}
	a, b := run(scene), run(restored)
	if !reflect.DeepEqual(a, b) {
		t.Fatal("restored scene must reproduce identical frames")
	}
}
