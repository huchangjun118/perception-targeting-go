package perception

import (
	"sort"
	"sync"
)

// WorldConfig describes the whole simulation.
type WorldConfig struct {
	TickDuration float64           `json:"tickDuration"`
	Seed         int64             `json:"seed"`
	Entities     []Entity          `json:"entities"`
	Obstacles    []Obstacle        `json:"obstacles"`
	Perceivers   []PerceiverConfig `json:"perceivers"`
	Claims       ClaimsConfig      `json:"claims"`
	Workers      int               `json:"workers"` // 0 = sequential perception
}

// World advances entities and perceivers deterministically.
//
// Determinism contract: with the same config (including Seed) and the same
// sequence of external mutations, Step produces byte-identical snapshots and
// claim outcomes regardless of the Workers setting or goroutine scheduling.
// This holds because (a) each perceiver has a private RNG seeded from
// (Seed, perceiver ID), (b) perception is read-only w.r.t. shared state, and
// (c) claims are resolved sequentially in perceiver-ID order after
// perception completes.
type World struct {
	cfg        WorldConfig
	tick       int64
	entities   []*Entity
	perceivers []*Perceiver // sorted by ID
	claims     *ClaimRegistry
	history    map[string][]*Snapshot // snapshots per perceiver ID
}

// NewWorld builds a world from a config. The config should be validated with
// ValidateConfig (or loaded via LoadConfig) first.
func NewWorld(cfg WorldConfig) *World {
	w := &World{
		cfg:     cfg,
		claims:  NewClaimRegistry(cfg.Claims),
		history: map[string][]*Snapshot{},
	}
	ents := make([]Entity, len(cfg.Entities))
	copy(ents, cfg.Entities)
	sort.Slice(ents, func(i, j int) bool { return ents[i].ID < ents[j].ID })
	for i := range ents {
		w.entities = append(w.entities, &ents[i])
	}
	pcs := make([]PerceiverConfig, len(cfg.Perceivers))
	copy(pcs, cfg.Perceivers)
	sort.Slice(pcs, func(i, j int) bool { return pcs[i].ID < pcs[j].ID })
	for _, pc := range pcs {
		w.perceivers = append(w.perceivers, NewPerceiver(pc, cfg.Seed))
	}
	return w
}

func (w *World) Tick() int64 { return w.tick }

// Claims exposes the claim registry.
func (w *World) Claims() *ClaimRegistry { return w.claims }

// Entity returns the live entity with the given ID, or nil. Callers may
// mutate it (e.g. velocity) between steps; such mutations are part of the
// recorded input sequence for replay purposes.
func (w *World) Entity(id string) *Entity {
	for _, e := range w.entities {
		if e.ID == id {
			return e
		}
	}
	return nil
}

// Step advances the world by one tick: entities move, due perceivers perceive
// (possibly in parallel), then claims are resolved in perceiver-ID order.
func (w *World) Step() {
	w.tick++
	dt := w.cfg.TickDuration
	if dt <= 0 {
		dt = 1
	}
	for _, e := range w.entities {
		e.Pos = e.Pos.Add(e.Vel.Scale(dt))
	}
	w.claims.Expire(w.tick)

	// Phase 1: perception. Pure w.r.t. shared state → safe to parallelize.
	due := make([]int, 0, len(w.perceivers))
	for i, p := range w.perceivers {
		if p.Due(w.tick, dt) {
			due = append(due, i)
		}
	}
	snaps := make([]*Snapshot, len(w.perceivers))
	workers := w.cfg.Workers
	if workers <= 1 || len(due) <= 1 {
		for _, i := range due {
			snaps[i] = w.perceivers[i].Perceive(w.tick, dt, w.entities, w.cfg.Obstacles)
		}
	} else {
		if workers > len(due) {
			workers = len(due)
		}
		jobs := make(chan int, len(due))
		var wg sync.WaitGroup
		for k := 0; k < workers; k++ {
			wg.Add(1)
			go func() {
				defer wg.Done()
				for i := range jobs {
					snaps[i] = w.perceivers[i].Perceive(w.tick, dt, w.entities, w.cfg.Obstacles)
				}
			}()
		}
		for _, i := range due {
			jobs <- i
		}
		close(jobs)
		wg.Wait()
	}

	// Phase 2: claim resolution, sequentially in perceiver-ID order.
	for _, i := range due {
		p := w.perceivers[i]
		snap := snaps[i]
		if snap.Chosen == "" {
			p.releaseClaim(w.claims)
		} else {
			if p.currentClaim != "" && p.currentClaim != snap.Chosen {
				p.releaseClaim(w.claims)
			}
			priority := 0.0
			for _, s := range snap.Scores {
				if s.TargetID == snap.Chosen {
					priority = s.Total
					break
				}
			}
			if w.claims.TryClaim(snap.Chosen, p.ID(), priority, w.tick) {
				p.currentClaim = snap.Chosen
				snap.Claimed = true
			}
		}
		w.history[p.ID()] = append(w.history[p.ID()], snap)
	}
}

// Run advances the world n ticks.
func (w *World) Run(n int) {
	for i := 0; i < n; i++ {
		w.Step()
	}
}

// History returns the recorded snapshots of a perceiver in tick order.
func (w *World) History(perceiverID string) []*Snapshot {
	return w.history[perceiverID]
}
