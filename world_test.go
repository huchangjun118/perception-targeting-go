package perception

import (
	"math"
	"reflect"
	"testing"
)

func testScoringCfg() ScoringConfig {
	return ScoringConfig{WDistance: 1, WHeading: 0.5, WVisibility: 1, WThreat: 2, MaxDistance: 200, VisNorm: 20}
}

func mkPerceiver(id string, x, y float64) PerceiverConfig {
	return PerceiverConfig{
		ID: id, Pos: Vec2{x, y}, Heading: 0,
		Sensors: []SensorConfig{
			{ID: "cam", FOV: math.Pi, Range: 150, RateHz: 10, NoisePos: 0.3, DropProb: 0.2},
			{ID: "radar", FOV: 2 * math.Pi, Range: 100, RateHz: 5, NoisePos: 1.0},
		},
		Memory:  MemoryConfig{TTLTicks: 15},
		Scoring: testScoringCfg(),
	}
}

func testWorldConfig(seed int64, workers int) WorldConfig {
	return WorldConfig{
		TickDuration: 0.1,
		Seed:         seed,
		Entities: []Entity{
			{ID: "e1", Pos: Vec2{40, 5}, Vel: Vec2{1, 0}, Radius: 1, Threat: 0.8},
			{ID: "e2", Pos: Vec2{60, -10}, Vel: Vec2{-0.5, 0.2}, Radius: 1.5, Threat: 0.4},
			{ID: "e3", Pos: Vec2{30, 30}, Vel: Vec2{0, -1}, Radius: 1, Threat: 0.6},
		},
		Obstacles: []Obstacle{
			{ID: "o1", Pos: Vec2{20, 2}, Radius: 3},
		},
		Perceivers: []PerceiverConfig{
			mkPerceiver("p1", 0, 0),
			mkPerceiver("p2", 10, -5),
			mkPerceiver("p3", -10, 10),
		},
		Claims:  ClaimsConfig{PreemptMargin: 0.1, HoldTicks: 3, TTLTicks: 10},
		Workers: workers,
	}
}

func runWorld(cfg WorldConfig, ticks int) map[string][]*Snapshot {
	w := NewWorld(cfg)
	w.Run(ticks)
	out := map[string][]*Snapshot{}
	for _, id := range []string{"p1", "p2", "p3"} {
		out[id] = w.History(id)
	}
	return out
}

func TestDeterminismAcrossWorkerCounts(t *testing.T) {
	seq := runWorld(testWorldConfig(42, 0), 60)
	par := runWorld(testWorldConfig(42, 8), 60)
	for id := range seq {
		if !reflect.DeepEqual(seq[id], par[id]) {
			t.Fatalf("perceiver %s: snapshots differ between sequential and parallel runs", id)
		}
	}
}

func TestDeterminismAcrossRepeats(t *testing.T) {
	a := runWorld(testWorldConfig(7, 4), 50)
	b := runWorld(testWorldConfig(7, 4), 50)
	for id := range a {
		if !reflect.DeepEqual(a[id], b[id]) {
			t.Fatalf("perceiver %s: same seed produced different snapshots", id)
		}
	}
}

func TestNoiseAndDropsOccurButReproduce(t *testing.T) {
	cfg := WorldConfig{
		TickDuration: 0.1,
		Seed:         99,
		Entities:     []Entity{{ID: "e1", Pos: Vec2{20, 0}, Radius: 1, Threat: 0.5}},
		Perceivers: []PerceiverConfig{{
			ID: "p1", Pos: Vec2{0, 0}, Heading: 0,
			Sensors: []SensorConfig{{ID: "cam", FOV: math.Pi / 2, Range: 100, RateHz: 10, NoisePos: 0.5, DropProb: 0.5}},
			Memory:  MemoryConfig{TTLTicks: 5},
			Scoring: testScoringCfg(),
		}},
		Claims: ClaimsConfig{PreemptMargin: 0.1, HoldTicks: 3, TTLTicks: 10},
	}
	w := NewWorld(cfg)
	w.Run(40)
	hist := w.History("p1")
	if len(hist) != 40 {
		t.Fatalf("expected 40 snapshots, got %d", len(hist))
	}
	zeroDet, noisy := 0, false
	for _, s := range hist {
		if len(s.Detections) == 0 {
			zeroDet++
			continue
		}
		// The noisy measurement must differ from the ground truth (20, 0).
		d := s.Detections[0]
		if d.Pos.X != 20 || d.Pos.Y != 0 {
			noisy = true
		}
	}
	if zeroDet == 0 {
		t.Fatal("expected some dropped frames with DropProb=0.5")
	}
	if !noisy {
		t.Fatal("expected noisy measurements to differ from ground truth")
	}
	// The whole noisy/dropping sequence must reproduce exactly with the same seed.
	w2 := NewWorld(cfg)
	w2.Run(40)
	if !reflect.DeepEqual(hist, w2.History("p1")) {
		t.Fatal("noisy sequence did not reproduce with the same seed")
	}
}

func TestClaimExclusivityPerTick(t *testing.T) {
	cfg := testWorldConfig(5, 4)
	w := NewWorld(cfg)
	percIDs := []string{"p1", "p2", "p3"}
	for i := 0; i < 60; i++ {
		w.Step()
		// Invariant: at the end of every tick each target has at most one
		// owner, and the owner's latest choice matches the claim.
		claims := w.Claims().Snapshot()
		seen := map[string]bool{}
		for _, c := range claims {
			if seen[c.TargetID] {
				t.Fatalf("tick %d: target %s owned by two perceivers", w.Tick(), c.TargetID)
			}
			seen[c.TargetID] = true
			found := false
			for _, id := range percIDs {
				if id != c.OwnerID {
					continue
				}
				hist := w.History(id)
				if len(hist) > 0 && hist[len(hist)-1].Chosen == c.TargetID {
					found = true
				}
			}
			if !found {
				t.Fatalf("tick %d: claim %+v not backed by owner's latest choice", w.Tick(), c)
			}
		}
	}
	// The scenario must actually exercise contention: more than one perceiver
	// chose the same target at the same tick at least once.
	type tickTarget struct {
		tick   int64
		target string
	}
	choosers := map[tickTarget][]string{}
	for _, id := range percIDs {
		for _, s := range w.History(id) {
			if s.Chosen != "" {
				k := tickTarget{s.Tick, s.Chosen}
				choosers[k] = append(choosers[k], id)
			}
		}
	}
	contended := 0
	for _, ids := range choosers {
		if len(ids) > 1 {
			contended++
		}
	}
	if contended == 0 {
		t.Fatal("scenario never had two perceivers choose the same target")
	}
}

func TestSlowPerceiverDoesNotBlockOthers(t *testing.T) {
	cfg := testWorldConfig(11, 0)
	// Make p3 very slow: 1 Hz -> updates every 10 ticks.
	cfg.Perceivers[2].Sensors = []SensorConfig{
		{ID: "slow", FOV: math.Pi, Range: 150, RateHz: 1, NoisePos: 0.1},
	}
	w := NewWorld(cfg)
	w.Run(30)
	if got := len(w.History("p1")); got != 30 {
		t.Fatalf("fast perceiver produced %d snapshots, want 30", got)
	}
	if got := len(w.History("p3")); got != 3 {
		t.Fatalf("slow perceiver produced %d snapshots, want 3", got)
	}
}

func TestMemoryBridgesOcclusion(t *testing.T) {
	// One perceiver, one target that passes behind a wide obstacle.
	cfg := WorldConfig{
		TickDuration: 0.1,
		Seed:         1,
		Entities: []Entity{
			{ID: "e1", Pos: Vec2{15, 0}, Vel: Vec2{0, 3}, Radius: 1, Threat: 0.5},
		},
		Obstacles: []Obstacle{{ID: "o1", Pos: Vec2{15, 6}, Radius: 2}},
		Perceivers: []PerceiverConfig{{
			ID: "p1", Pos: Vec2{0, 0}, Heading: 0,
			Sensors: []SensorConfig{{ID: "cam", FOV: 2 * math.Pi, Range: 200, RateHz: 10}},
			Memory:  MemoryConfig{TTLTicks: 8},
			Scoring: testScoringCfg(),
		}},
		Claims: ClaimsConfig{PreemptMargin: 0.1, HoldTicks: 3, TTLTicks: 10},
	}
	w := NewWorld(cfg)
	sawMemory := false
	for i := 0; i < 40; i++ {
		w.Step()
	}
	for _, s := range w.History("p1") {
		for _, tr := range s.Tracks {
			if tr.FromMemory {
				sawMemory = true
			}
		}
	}
	if !sawMemory {
		t.Fatal("expected memory to retain the target while occluded")
	}
}
