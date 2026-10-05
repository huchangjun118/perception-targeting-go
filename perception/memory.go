package perception

import (
	"sort"

	"github.com/huchangjun118/perception-targeting-go/geom"
)

// MemoryConfig 感知记忆配置。
type MemoryConfig struct {
	TTLTicks uint64  `json:"ttl_ticks"` // 记忆有效期（tick 数）
	Decay    float64 `json:"decay"`     // 每 tick 置信度衰减系数 (0,1]
}

// MemoryEntry 单目标记忆条目。
type MemoryEntry struct {
	TargetID     string
	Position     geom.Vec2
	Visibility   Visibility
	Confidence   float64
	LastSeenTick uint64
	Threat       float64
}

// Expired 判断条目在 now 是否已过期。
// 规则确定：now - LastSeenTick >= ttl 即过期（ttl 为 0 时仅当前 tick 有效）。
func (e MemoryEntry) Expired(now uint64, ttl uint64) bool {
	return now-e.LastSeenTick >= ttl
}

// Memory 感知记忆：目标离开视野后按有效期保留。
type Memory struct {
	cfg     MemoryConfig
	entries map[string]MemoryEntry
}

func NewMemory(cfg MemoryConfig) *Memory {
	return &Memory{cfg: cfg, entries: make(map[string]MemoryEntry)}
}

// Update 用当前快照更新记忆，并对未观测条目施加衰减。
// 过期条目先清除，迟到信息（LastSeenTick 早于现有条目）被拒绝。
func (m *Memory) Update(snap Snapshot, threats map[string]float64) {
	now := snap.Tick
	// 清除已过期条目。
	for id, e := range m.entries {
		if e.Expired(now, m.cfg.TTLTicks) {
			delete(m.entries, id)
		}
	}
	seen := make(map[string]bool, len(snap.Targets))
	for _, p := range snap.Targets {
		seen[p.TargetID] = true
		old, ok := m.entries[p.TargetID]
		if ok && p.LastSeenTick < old.LastSeenTick {
			continue // 迟到信息不得覆盖较新的记忆
		}
		m.entries[p.TargetID] = MemoryEntry{
			TargetID:     p.TargetID,
			Position:     p.Position,
			Visibility:   p.Visibility,
			Confidence:   p.Confidence,
			LastSeenTick: p.LastSeenTick,
			Threat:       threats[p.TargetID],
		}
	}
	// 未观测条目置信度衰减。
	for id, e := range m.entries {
		if seen[id] {
			continue
		}
		e.Confidence *= m.cfg.Decay
		m.entries[id] = e
	}
}

// Get 返回 now 时刻仍有效的记忆条目；过期条目视为无效。
func (m *Memory) Get(targetID string, now uint64) (MemoryEntry, bool) {
	e, ok := m.entries[targetID]
	if !ok || e.Expired(now, m.cfg.TTLTicks) {
		return MemoryEntry{}, false
	}
	return e, true
}

// Valid 返回 now 时刻所有未过期条目（按 TargetID 排序）。
func (m *Memory) Valid(now uint64) []MemoryEntry {
	var out []MemoryEntry
	for _, e := range m.entries {
		if !e.Expired(now, m.cfg.TTLTicks) {
			out = append(out, e)
		}
	}
	sort.Slice(out, func(i, j int) bool { return out[i].TargetID < out[j].TargetID })
	return out
}
