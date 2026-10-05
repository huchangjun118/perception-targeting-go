package perception

import "testing"

func memEntity(id string, x float64) *Entity {
	return &Entity{ID: id, Pos: Vec2{x, 0}, Radius: 1, Threat: 0.5}
}

func TestMemoryRetainAndExpire(t *testing.T) {
	m := NewMemory(MemoryConfig{TTLTicks: 3})
	m.Observe(1, memEntity("a", 10), Vec2{10, 0})

	// Target disappears; memory must retain it within the TTL.
	for tick := int64(2); tick <= 4; tick++ {
		m.Advance(tick)
		tracks := m.ValidTracks(tick)
		if len(tracks) != 1 {
			t.Fatalf("tick %d: expected 1 valid track, got %d", tick, len(tracks))
		}
		if !tracks[0].FromMemory {
			t.Fatalf("tick %d: expected FromMemory track", tick)
		}
		if tracks[0].Confidence >= 1 {
			t.Fatalf("tick %d: expected decayed confidence, got %v", tick, tracks[0].Confidence)
		}
	}
	// After the TTL the track expires and must not participate in decisions.
	m.Advance(5)
	if n := len(m.ValidTracks(5)); n != 0 {
		t.Fatalf("expected 0 valid tracks after TTL, got %d", n)
	}
}

func TestMemoryConfidenceDecaysMonotonically(t *testing.T) {
	m := NewMemory(MemoryConfig{TTLTicks: 10})
	m.Observe(1, memEntity("a", 10), Vec2{10, 0})
	prev := 1.0
	for tick := int64(2); tick <= 11; tick++ {
		m.Advance(tick)
		tracks := m.ValidTracks(tick)
		if len(tracks) == 0 {
			break
		}
		if tracks[0].Confidence > prev {
			t.Fatalf("confidence increased: %v -> %v", prev, tracks[0].Confidence)
		}
		prev = tracks[0].Confidence
	}
}

func TestMemoryRejectsStaleObservation(t *testing.T) {
	m := NewMemory(MemoryConfig{TTLTicks: 10})
	m.Observe(5, memEntity("a", 10), Vec2{10, 0})
	// A late observation from an older tick must be rejected.
	m.Observe(3, memEntity("a", 99), Vec2{99, 0})
	tr := m.ValidTracks(5)[0]
	if tr.Pos.X != 10 || tr.LastSeenTick != 5 {
		t.Fatalf("stale observation overwrote track: %+v", tr)
	}
}

func TestMemoryReobserveRefreshes(t *testing.T) {
	m := NewMemory(MemoryConfig{TTLTicks: 3})
	m.Observe(1, memEntity("a", 10), Vec2{10, 0})
	m.Advance(2)
	m.Advance(3)
	m.Observe(4, memEntity("a", 12), Vec2{12, 0}) // re-observed before expiry
	tr := m.ValidTracks(4)[0]
	if tr.Confidence != 1 || tr.FromMemory || tr.Pos.X != 12 {
		t.Fatalf("expected refreshed track, got %+v", tr)
	}
	if tr.VisibleTicks != 2 {
		t.Fatalf("expected 2 visible ticks, got %d", tr.VisibleTicks)
	}
}

func TestValidTracksDeterministicOrder(t *testing.T) {
	m := NewMemory(MemoryConfig{TTLTicks: 5})
	m.Observe(1, memEntity("b", 0), Vec2{})
	m.Observe(1, memEntity("a", 0), Vec2{})
	m.Observe(1, memEntity("c", 0), Vec2{})
	tracks := m.ValidTracks(1)
	if tracks[0].TargetID != "a" || tracks[1].TargetID != "b" || tracks[2].TargetID != "c" {
		t.Fatalf("tracks not sorted by ID: %v", tracks)
	}
}
