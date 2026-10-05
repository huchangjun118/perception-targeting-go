package perception

import (
	"testing"

	"github.com/huchangjun118/perception-targeting-go/geom"
)

func memCfg() MemoryConfig { return MemoryConfig{TTLTicks: 3, Decay: 0.5} }

func snapOf(tick uint64, ids ...string) Snapshot {
	s := Snapshot{PerceiverID: "a", Tick: tick}
	for _, id := range ids {
		s.Targets = append(s.Targets, TargetPercept{
			TargetID: id, Position: geom.Vec2{X: 1, Y: 1},
			Visibility: Full, Confidence: 1, LastSeenTick: tick,
		})
	}
	return s
}

func TestMemoryExpiry(t *testing.T) {
	m := NewMemory(memCfg())
	m.Update(snapOf(0, "t1"), nil)
	// tick 0,1,2 有效；tick 3 过期（now-last >= ttl）
	for tick := uint64(0); tick < 3; tick++ {
		if _, ok := m.Get("t1", tick); !ok {
			t.Fatalf("tick %d: expected valid memory", tick)
		}
	}
	if _, ok := m.Get("t1", 3); ok {
		t.Fatal("tick 3: expected expired memory")
	}
	// 过期条目不参与 Valid
	m.Update(snapOf(5), nil) // tick 5：t1 已过期被清除
	if got := m.Valid(5); len(got) != 0 {
		t.Fatalf("expected no valid entries, got %+v", got)
	}
}

func TestMemoryDecay(t *testing.T) {
	m := NewMemory(memCfg())
	m.Update(snapOf(0, "t1"), nil)
	m.Update(snapOf(1), nil) // t1 消失一帧
	e, ok := m.Get("t1", 1)
	if !ok {
		t.Fatal("expected retained memory")
	}
	if e.Confidence != 0.5 {
		t.Fatalf("expected decayed confidence 0.5, got %v", e.Confidence)
	}
}

func TestMemoryRejectsLateInfo(t *testing.T) {
	m := NewMemory(memCfg())
	m.Update(snapOf(2, "t1"), nil)
	// 迟到的旧快照（LastSeenTick=1）不得覆盖
	late := Snapshot{PerceiverID: "a", Tick: 2, Targets: []TargetPercept{{
		TargetID: "t1", Position: geom.Vec2{X: 9, Y: 9},
		Visibility: Partial, Confidence: 0.1, LastSeenTick: 1,
	}}}
	m.Update(late, nil)
	e, _ := m.Get("t1", 2)
	if e.Position != (geom.Vec2{X: 1, Y: 1}) || e.LastSeenTick != 2 {
		t.Fatalf("late info must not overwrite newer memory: %+v", e)
	}
}

func TestMemoryThreatRecorded(t *testing.T) {
	m := NewMemory(memCfg())
	m.Update(snapOf(0, "t1"), map[string]float64{"t1": 0.8})
	e, _ := m.Get("t1", 0)
	if e.Threat != 0.8 {
		t.Fatalf("expected threat 0.8, got %v", e.Threat)
	}
}
