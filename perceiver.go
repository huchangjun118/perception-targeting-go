package perception

import (
	"hash/fnv"
	"math"
	"math/rand"
	"sort"
)

// PerceiverConfig bundles everything needed to build a perceiver.
type PerceiverConfig struct {
	ID      string         `json:"id"`
	Pos     Vec2           `json:"pos"`
	Heading float64        `json:"heading"`
	Sensors []SensorConfig `json:"sensors"`
	Memory  MemoryConfig   `json:"memory"`
	Scoring ScoringConfig  `json:"scoring"`
}

// Perceiver fuses its sensors, maintains memory and proposes targets.
// All of its mutable state (RNG, memory) is private, so different
// perceivers may run concurrently without synchronization.
type Perceiver struct {
	cfg          PerceiverConfig
	memory       *Memory
	rng          *rand.Rand
	currentClaim string // target currently claimed by this perceiver
}

// NewPerceiver builds a perceiver. The RNG is seeded deterministically from
// the world seed and the perceiver ID, so replays with the same seed and ID
// produce identical noise/drop sequences regardless of scheduling.
func NewPerceiver(cfg PerceiverConfig, seed int64) *Perceiver {
	h := fnv.New64a()
	h.Write([]byte(cfg.ID))
	rng := rand.New(rand.NewSource(seed ^ int64(h.Sum64())))
	return &Perceiver{cfg: cfg, memory: NewMemory(cfg.Memory), rng: rng}
}

func (p *Perceiver) ID() string { return p.cfg.ID }

// sensorPeriod converts a sensor's update rate into an integer tick period.
func sensorPeriod(s SensorConfig, tickDuration float64) int64 {
	if s.RateHz <= 0 || tickDuration <= 0 {
		return 1
	}
	period := int64(math.Round(1 / (s.RateHz * tickDuration)))
	if period < 1 {
		period = 1
	}
	return period
}

// Due reports whether any sensor updates at the given tick.
func (p *Perceiver) Due(tick int64, tickDuration float64) bool {
	for _, s := range p.cfg.Sensors {
		if tick%sensorPeriod(s, tickDuration) == 0 {
			return true
		}
	}
	return false
}

// Perceive computes the fused snapshot for the given tick. entities must be
// sorted by ID (the world guarantees this). The method only touches private
// state, so perceivers may perceive concurrently.
//
// The returned snapshot's Chosen field holds the proposed target; claim
// resolution happens later (deterministically, in the world), which fills in
// the Claimed field.
func (p *Perceiver) Perceive(tick int64, tickDuration float64, entities []*Entity, obstacles []Obstacle) *Snapshot {
	snap := &Snapshot{PerceiverID: p.cfg.ID, Tick: tick}

	// 1. Run each due sensor in config order (deterministic).
	for _, s := range p.cfg.Sensors {
		if tick%sensorPeriod(s, tickDuration) != 0 {
			continue
		}
		for _, e := range entities {
			vis := AssessVisibility(p.cfg.Pos, p.cfg.Heading, s, e, obstacles, entities)
			if vis.Class == NotVisible {
				continue
			}
			if s.DropProb > 0 && p.rng.Float64() < s.DropProb {
				continue // dropped frame
			}
			pos := e.Pos
			if s.NoisePos > 0 {
				pos = Vec2{
					X: e.Pos.X + p.rng.NormFloat64()*s.NoisePos,
					Y: e.Pos.Y + p.rng.NormFloat64()*s.NoisePos,
				}
			}
			snap.Detections = append(snap.Detections, Detection{
				SensorID:   s.ID,
				TargetID:   e.ID,
				Pos:        pos,
				Visibility: vis.Class,
				Fraction:   vis.VisibleFraction,
			})
		}
	}

	// 2. Fuse detections of the same target across sensors.
	//    Positions are averaged with inverse-noise-variance weights; the
	//    iteration order is fixed by sensor config order and entity ID order.
	type agg struct {
		sumW, sumX, sumY float64
		sensors          []string
		vis              VisibilityClass
		frac             float64
	}
	aggs := map[string]*agg{}
	var order []string
	for _, d := range snap.Detections {
		a, ok := aggs[d.TargetID]
		if !ok {
			a = &agg{}
			aggs[d.TargetID] = a
			order = append(order, d.TargetID)
		}
		w := 1.0
		for _, s := range p.cfg.Sensors {
			if s.ID == d.SensorID && s.NoisePos > 0 {
				w = 1 / (s.NoisePos * s.NoisePos)
				break
			}
		}
		a.sumW += w
		a.sumX += d.Pos.X * w
		a.sumY += d.Pos.Y * w
		a.sensors = append(a.sensors, d.SensorID)
		if d.Visibility > a.vis {
			a.vis = d.Visibility
		}
		if d.Fraction > a.frac {
			a.frac = d.Fraction
		}
	}
	sort.Strings(order)
	for _, id := range order {
		a := aggs[id]
		sort.Strings(a.sensors)
		snap.Fused = append(snap.Fused, FusedTarget{
			TargetID:   id,
			Pos:        Vec2{X: a.sumX / a.sumW, Y: a.sumY / a.sumW},
			Sensors:    a.sensors,
			Visibility: a.vis,
			Fraction:   a.frac,
		})
	}

	// 3. Update memory with fused observations, then decay.
	for _, f := range snap.Fused {
		for _, e := range entities {
			if e.ID == f.TargetID {
				p.memory.Observe(tick, e, f.Pos)
				break
			}
		}
	}
	p.memory.Advance(tick)

	// 4. Score valid tracks and propose the best target.
	tracks := p.memory.ValidTracks(tick)
	for _, t := range tracks {
		snap.Tracks = append(snap.Tracks, *t)
		snap.Scores = append(snap.Scores, ScoreTarget(p.cfg.Scoring, p.cfg.Pos, p.cfg.Heading, t))
	}
	if best := SelectTarget(snap.Scores); best != nil {
		snap.Chosen = best.TargetID
	}
	return snap
}

// releaseClaim releases the perceiver's current claim if it no longer
// matches the chosen target. Called by the world during claim resolution.
func (p *Perceiver) releaseClaim(reg *ClaimRegistry) {
	if p.currentClaim != "" {
		reg.Release(p.currentClaim, p.cfg.ID)
		p.currentClaim = ""
	}
}
