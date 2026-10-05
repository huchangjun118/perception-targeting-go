package perception

import (
	"sort"
	"sync"
)

// Claim records that a perceiver occupies a target.
type Claim struct {
	TargetID string
	OwnerID  string
	Priority float64
	Tick     int64
}

// ClaimsConfig controls preemption and expiry of claims.
type ClaimsConfig struct {
	PreemptMargin float64 `json:"preemptMargin"` // priority margin needed to preempt
	HoldTicks     int64   `json:"holdTicks"`     // min ticks a claim is held before ID tie-break preemption
	TTLTicks      int64   `json:"ttlTicks"`      // claims expire after this many ticks without refresh
}

// ClaimRegistry tracks target ownership. Its rules are arrival-order
// independent, so concurrent scheduling cannot change the final outcome:
//
//  1. An unclaimed or expired target goes to the requester.
//  2. A clearly higher priority (greater than current + PreemptMargin) always
//     preempts.
//  3. Once a claim is older than HoldTicks, a requester with roughly equal
//     priority (within PreemptMargin) and a lexicographically smaller owner
//     ID preempts — a deterministic tie-break that does not depend on
//     arrival order.
//  4. Otherwise the current holder keeps the target.
type ClaimRegistry struct {
	mu     sync.Mutex
	cfg    ClaimsConfig
	claims map[string]Claim // by target ID
}

func NewClaimRegistry(cfg ClaimsConfig) *ClaimRegistry {
	return &ClaimRegistry{cfg: cfg, claims: map[string]Claim{}}
}

func (r *ClaimRegistry) expired(c Claim, tick int64) bool {
	return r.cfg.TTLTicks > 0 && tick-c.Tick > r.cfg.TTLTicks
}

// TryClaim attempts to claim a target. Returns true if the caller owns the
// target after the call.
func (r *ClaimRegistry) TryClaim(targetID, ownerID string, priority float64, tick int64) bool {
	r.mu.Lock()
	defer r.mu.Unlock()
	c, ok := r.claims[targetID]
	if !ok || r.expired(c, tick) {
		r.claims[targetID] = Claim{TargetID: targetID, OwnerID: ownerID, Priority: priority, Tick: tick}
		return true
	}
	if c.OwnerID == ownerID {
		// Refresh own claim.
		c.Priority = priority
		c.Tick = tick
		r.claims[targetID] = c
		return true
	}
	if priority > c.Priority+r.cfg.PreemptMargin {
		r.claims[targetID] = Claim{TargetID: targetID, OwnerID: ownerID, Priority: priority, Tick: tick}
		return true
	}
	if tick-c.Tick >= r.cfg.HoldTicks &&
		priority >= c.Priority-r.cfg.PreemptMargin &&
		ownerID < c.OwnerID {
		r.claims[targetID] = Claim{TargetID: targetID, OwnerID: ownerID, Priority: priority, Tick: tick}
		return true
	}
	return false
}

// Release removes a claim, but only if ownerID currently owns it.
func (r *ClaimRegistry) Release(targetID, ownerID string) {
	r.mu.Lock()
	defer r.mu.Unlock()
	if c, ok := r.claims[targetID]; ok && c.OwnerID == ownerID {
		delete(r.claims, targetID)
	}
}

// Owner returns the current owner of a target, or "" if unclaimed.
func (r *ClaimRegistry) Owner(targetID string) string {
	r.mu.Lock()
	defer r.mu.Unlock()
	if c, ok := r.claims[targetID]; ok {
		return c.OwnerID
	}
	return ""
}

// Expire drops claims older than the TTL at the given tick.
func (r *ClaimRegistry) Expire(tick int64) {
	r.mu.Lock()
	defer r.mu.Unlock()
	for id, c := range r.claims {
		if r.expired(c, tick) {
			delete(r.claims, id)
		}
	}
}

// Snapshot returns a deterministic copy of all claims (sorted by target ID).
func (r *ClaimRegistry) Snapshot() []Claim {
	r.mu.Lock()
	defer r.mu.Unlock()
	out := make([]Claim, 0, len(r.claims))
	for _, c := range r.claims {
		out = append(out, c)
	}
	sort.Slice(out, func(i, j int) bool { return out[i].TargetID < out[j].TargetID })
	return out
}
