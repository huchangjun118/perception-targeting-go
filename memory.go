package perception

import "sort"

// Track is a remembered perception of a target.
type Track struct {
	TargetID      string
	Pos           Vec2
	Vel           Vec2
	Threat        float64
	FirstSeenTick int64
	LastSeenTick  int64
	VisibleTicks  int64 // cumulative ticks the target was directly observed
	Confidence    float64
	FromMemory    bool // true if not directly observed at the current tick
}

// MemoryConfig controls retention and decay of tracks.
type MemoryConfig struct {
	TTLTicks int64 `json:"ttlTicks"` // ticks a track is kept after its last observation
}

// Memory holds the perceived tracks of one perceiver.
type Memory struct {
	cfg    MemoryConfig
	tracks map[string]*Track
}

func NewMemory(cfg MemoryConfig) *Memory {
	if cfg.TTLTicks < 1 {
		cfg.TTLTicks = 1
	}
	return &Memory{cfg: cfg, tracks: map[string]*Track{}}
}

// Observe records a direct observation of a target at the given tick.
// Stale observations (older than the stored track) are rejected so that
// late-arriving information never overwrites fresher state.
func (m *Memory) Observe(tick int64, e *Entity, pos Vec2) {
	t, ok := m.tracks[e.ID]
	if ok {
		if tick < t.LastSeenTick {
			return // stale: reject
		}
		if tick > t.LastSeenTick {
			t.VisibleTicks++
		}
		t.Pos = pos
		t.Vel = e.Vel
		t.Threat = e.Threat
		t.LastSeenTick = tick
		t.Confidence = 1
		t.FromMemory = false
		return
	}
	m.tracks[e.ID] = &Track{
		TargetID:      e.ID,
		Pos:           pos,
		Vel:           e.Vel,
		Threat:        e.Threat,
		FirstSeenTick: tick,
		LastSeenTick:  tick,
		VisibleTicks:  1,
		Confidence:    1,
	}
}

// Advance decays confidences and expires old tracks at the given tick.
// Confidence decays linearly from 1 to 0 across the TTL; a track whose age
// exceeds the TTL is expired and removed, so it can no longer influence
// decisions.
func (m *Memory) Advance(tick int64) {
	for id, t := range m.tracks {
		age := tick - t.LastSeenTick
		switch {
		case age < 0:
			// Should not happen; treat as expired defensively.
			delete(m.tracks, id)
		case age == 0:
			t.Confidence = 1
			t.FromMemory = false
		case age > m.cfg.TTLTicks:
			delete(m.tracks, id)
		default:
			t.Confidence = 1 - float64(age)/float64(m.cfg.TTLTicks+1)
			t.FromMemory = true
		}
	}
}

// ValidTracks returns the non-expired tracks at the current tick, in
// deterministic order (sorted by target ID).
func (m *Memory) ValidTracks(tick int64) []*Track {
	out := make([]*Track, 0, len(m.tracks))
	for _, t := range m.tracks {
		if tick-t.LastSeenTick > m.cfg.TTLTicks {
			continue // expired: never participates in decisions
		}
		out = append(out, t)
	}
	sort.Slice(out, func(i, j int) bool { return out[i].TargetID < out[j].TargetID })
	return out
}
