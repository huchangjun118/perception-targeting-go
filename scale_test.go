package perception

import (
	"fmt"
	"math"
	"reflect"
	"testing"
)

// scaleConfig builds a larger scenario: many perceivers and entities.
func scaleConfig(seed int64, nPerceivers, nEntities, workers int) WorldConfig {
	cfg := WorldConfig{
		TickDuration: 0.1,
		Seed:         seed,
		Claims:       ClaimsConfig{PreemptMargin: 0.1, HoldTicks: 3, TTLTicks: 10},
		Workers:      workers,
	}
	for i := 0; i < nEntities; i++ {
		cfg.Entities = append(cfg.Entities, Entity{
			ID:     fmt.Sprintf("e%03d", i),
			Pos:    Vec2{float64(50 + (i%10)*15), float64(-100 + (i/10)*20)},
			Vel:    Vec2{float64((i%3)-1) * 0.7, float64(((i+1)%3)-1) * 0.5},
			Radius: 1 + float64(i%3)*0.5,
			Threat: float64(i%10) / 10,
		})
	}
	for i := 0; i < 5; i++ {
		cfg.Obstacles = append(cfg.Obstacles, Obstacle{
			ID:     fmt.Sprintf("o%d", i),
			Pos:    Vec2{float64(60 + i*30), float64(-50 + i*25)},
			Radius: 4,
		})
	}
	scoring := ScoringConfig{WDistance: 1, WHeading: 0.5, WVisibility: 1, WThreat: 2, MaxDistance: 400, VisNorm: 20}
	for i := 0; i < nPerceivers; i++ {
		cfg.Perceivers = append(cfg.Perceivers, PerceiverConfig{
			ID:      fmt.Sprintf("p%03d", i),
			Pos:     Vec2{float64((i%8)*40 - 140), float64((i/8)*60 - 150)},
			Heading: float64(i%8) * math.Pi / 4,
			Sensors: []SensorConfig{
				{ID: "cam", FOV: math.Pi / 2, Range: 180, RateHz: 10, NoisePos: 0.4, DropProb: 0.1},
				{ID: "radar", FOV: 2 * math.Pi, Range: 120, RateHz: 2, NoisePos: 1.5},
			},
			Memory:  MemoryConfig{TTLTicks: 12},
			Scoring: scoring,
		})
	}
	return cfg
}

func TestScaleDeterministicAcrossWorkerCounts(t *testing.T) {
	const ticks = 40
	seq := NewWorld(scaleConfig(123, 40, 80, 0))
	seq.Run(ticks)
	par := NewWorld(scaleConfig(123, 40, 80, 8))
	par.Run(ticks)
	for i := 0; i < 40; i++ {
		id := fmt.Sprintf("p%03d", i)
		a, b := seq.History(id), par.History(id)
		if !reflect.DeepEqual(a, b) {
			t.Fatalf("perceiver %s: parallel and sequential runs diverge", id)
		}
		if len(a) == 0 {
			t.Fatalf("perceiver %s produced no snapshots", id)
		}
	}
	// Claims must also end in the same state.
	if !reflect.DeepEqual(seq.Claims().Snapshot(), par.Claims().Snapshot()) {
		t.Fatal("claim registries diverged between sequential and parallel runs")
	}
}

func BenchmarkWorldStep(b *testing.B) {
	cfg := scaleConfig(1, 64, 128, 8)
	w := NewWorld(cfg)
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		w.Step()
	}
}
