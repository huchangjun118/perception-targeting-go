package config

import (
	"encoding/json"
	"strings"
	"testing"

	"github.com/huchangjun118/perception-targeting-go/geom"
	"github.com/huchangjun118/perception-targeting-go/perception"
	"github.com/huchangjun118/perception-targeting-go/sim"
	"github.com/huchangjun118/perception-targeting-go/targeting"
)

func validScene() sim.Scene {
	return sim.Scene{
		StepHz: 10,
		Obstacles: []geom.Segment{
			{A: geom.Vec2{X: 5, Y: -1}, B: geom.Vec2{X: 5, Y: 1}},
		},
		Targets: []perception.Target{
			{ID: "t1", Pos: geom.Vec2{X: 10}, Radius: 1, Threat: 0.5},
		},
		Agents: []sim.Agent{{
			ID: "a1",
			Sensors: []perception.SensorConfig{
				{ID: "s1", FOVDeg: 180, Range: 50, RateHz: 10, NoiseStdPos: 0.1, DropProb: 0.1},
			},
			Weights:  targeting.ScoreWeights{Distance: 1, Threat: 1},
			MemCfg:   perception.MemoryConfig{TTLTicks: 5, Decay: 0.9},
			MaxRange: 100,
		}},
	}
}

func TestRoundTrip(t *testing.T) {
	data, err := Marshal(validScene())
	if err != nil {
		t.Fatalf("marshal: %v", err)
	}
	got, err := Unmarshal(data)
	if err != nil {
		t.Fatalf("unmarshal: %v", err)
	}
	if got.StepHz != 10 || len(got.Agents) != 1 || got.Agents[0].Sensors[0].ID != "s1" {
		t.Fatalf("round trip mismatch: %+v", got)
	}
}

func TestMigrateV1(t *testing.T) {
	// v1 数据缺少 drop_prob / decay / max_range 字段
	v1 := `{"version":1,"scene":{"step_hz":10,
		"targets":[{"id":"t1","pos":{"x":10,"y":0},"radius":1,"threat":0.5}],
		"agents":[{"id":"a1",
			"sensors":[{"id":"s1","fov_deg":180,"range":50,"rate_hz":10,"noise_std_pos":0.1}],
			"weights":{"distance":1},"memory":{"ttl_ticks":5}}]}}`
	s, err := Unmarshal([]byte(v1))
	if err != nil {
		t.Fatalf("v1 should migrate: %v", err)
	}
	if s.Agents[0].MaxRange != 100 {
		t.Fatalf("expected default max_range 100, got %v", s.Agents[0].MaxRange)
	}
	if s.Agents[0].MemCfg.Decay != 0.95 {
		t.Fatalf("expected default decay 0.95, got %v", s.Agents[0].MemCfg.Decay)
	}
}

func TestErrorKindsDistinct(t *testing.T) {
	// malformed
	_, err := Unmarshal([]byte("{not json"))
	kindOf := func(e error) ErrKind {
		if ce, ok := e.(*Error); ok {
			return ce.Kind
		}
		t.Fatalf("expected *Error, got %T", e)
		return ""
	}
	if kindOf(err) != ErrMalformed {
		t.Fatalf("expected malformed, got %v", err)
	}
	// unsupported version
	_, err = Unmarshal([]byte(`{"version":99,"scene":{}}`))
	if kindOf(err) != ErrUnsupported {
		t.Fatalf("expected unsupported, got %v", err)
	}
	// validation: 缺传感器
	bad := validScene()
	bad.Agents[0].Sensors = nil
	data, _ := json.Marshal(Envelope{Version: CurrentVersion, Scene: bad})
	_, err = Unmarshal(data)
	if kindOf(err) != ErrValidation {
		t.Fatalf("expected validation, got %v", err)
	}
}

func TestValidationCases(t *testing.T) {
	cases := []struct {
		name   string
		mut    func(*sim.Scene)
		substr string
	}{
		{"step hz", func(s *sim.Scene) { s.StepHz = 0 }, "step_hz"},
		{"dup agent", func(s *sim.Scene) { s.Agents = append(s.Agents, s.Agents[0]) }, "duplicate agent"},
		{"bad fov", func(s *sim.Scene) { s.Agents[0].Sensors[0].FOVDeg = 400 }, "fov_deg"},
		{"bad range", func(s *sim.Scene) { s.Agents[0].Sensors[0].Range = -1 }, "range"},
		{"bad rate", func(s *sim.Scene) { s.Agents[0].Sensors[0].RateHz = 0 }, "rate_hz"},
		{"bad noise", func(s *sim.Scene) { s.Agents[0].Sensors[0].NoiseStdPos = -0.1 }, "noise_std_pos"},
		{"bad drop", func(s *sim.Scene) { s.Agents[0].Sensors[0].DropProb = 1.5 }, "drop_prob"},
		{"dup sensor", func(s *sim.Scene) {
			s.Agents[0].Sensors = append(s.Agents[0].Sensors, s.Agents[0].Sensors[0])
		}, "duplicate sensor"},
		{"bad ttl", func(s *sim.Scene) { s.Agents[0].MemCfg.TTLTicks = 0 }, "ttl_ticks"},
		{"bad decay", func(s *sim.Scene) { s.Agents[0].MemCfg.Decay = 1.5 }, "decay"},
		{"bad threat", func(s *sim.Scene) { s.Targets[0].Threat = 2 }, "threat"},
		{"bad radius", func(s *sim.Scene) { s.Targets[0].Radius = 0 }, "radius"},
		{"dup target", func(s *sim.Scene) { s.Targets = append(s.Targets, s.Targets[0]) }, "duplicate target"},
		{"degenerate obstacle", func(s *sim.Scene) {
			s.Obstacles[0].B = s.Obstacles[0].A
		}, "degenerate"},
		{"no maxrange", func(s *sim.Scene) { s.Agents[0].MaxRange = 0 }, "max_range"},
	}
	for _, tc := range cases {
		s := validScene()
		tc.mut(&s)
		err := Validate(s)
		if err == nil {
			t.Fatalf("%s: expected error", tc.name)
		}
		ce, ok := err.(*Error)
		if !ok || ce.Kind != ErrValidation {
			t.Fatalf("%s: expected validation error, got %v", tc.name, err)
		}
		if !strings.Contains(ce.Detail, tc.substr) {
			t.Fatalf("%s: error %q missing %q", tc.name, ce.Detail, tc.substr)
		}
	}
}

func TestMarshalRejectsInvalid(t *testing.T) {
	s := validScene()
	s.StepHz = -1
	if _, err := Marshal(s); err == nil {
		t.Fatal("marshal must reject invalid scene")
	}
}
