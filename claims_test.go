package perception

import "testing"

func testRegistry() *ClaimRegistry {
	return NewClaimRegistry(ClaimsConfig{PreemptMargin: 0.1, HoldTicks: 3, TTLTicks: 10})
}

func TestExclusiveClaim(t *testing.T) {
	r := testRegistry()
	if !r.TryClaim("t1", "alice", 1.0, 0) {
		t.Fatal("first claim should succeed")
	}
	if r.TryClaim("t1", "bob", 1.0, 1) {
		t.Fatal("equal-priority claim within hold time must fail")
	}
	if got := r.Owner("t1"); got != "alice" {
		t.Fatalf("owner = %v, want alice", got)
	}
}

func TestPreemptByHigherPriority(t *testing.T) {
	r := testRegistry()
	r.TryClaim("t1", "alice", 1.0, 0)
	if !r.TryClaim("t1", "bob", 1.0+0.2, 1) {
		t.Fatal("clearly higher priority should preempt")
	}
	if got := r.Owner("t1"); got != "bob" {
		t.Fatalf("owner = %v, want bob", got)
	}
}

func TestTieBreakByIDAfterHold(t *testing.T) {
	r := testRegistry()
	r.TryClaim("t1", "bob", 1.0, 0)
	// Within hold time: even smaller ID cannot steal.
	if r.TryClaim("t1", "alice", 1.0, 2) {
		t.Fatal("claim within hold time must not be stolen on tie")
	}
	// After hold time: smaller ID wins the tie regardless of arrival order.
	if !r.TryClaim("t1", "alice", 1.0, 5) {
		t.Fatal("smaller ID should win tie after hold time")
	}
	if got := r.Owner("t1"); got != "alice" {
		t.Fatalf("owner = %v, want alice", got)
	}
}

func TestClaimOutcomeArrivalOrderIndependent(t *testing.T) {
	run := func(order []string) string {
		r := testRegistry()
		for tick, owner := range order {
			r.TryClaim("t1", owner, 1.0, int64(tick)*10) // spaced past hold time
		}
		return r.Owner("t1")
	}
	a := run([]string{"alice", "bob"})
	b := run([]string{"bob", "alice"})
	if a != b {
		t.Fatalf("outcome depends on arrival order: %v vs %v", a, b)
	}
	if a != "alice" {
		t.Fatalf("deterministic tie-break should favor alice, got %v", a)
	}
}

func TestRelease(t *testing.T) {
	r := testRegistry()
	r.TryClaim("t1", "alice", 1.0, 0)
	r.Release("t1", "bob") // not the owner: no-op
	if r.Owner("t1") != "alice" {
		t.Fatal("non-owner release must be a no-op")
	}
	r.Release("t1", "alice")
	if r.Owner("t1") != "" {
		t.Fatal("claim should be released")
	}
	if !r.TryClaim("t1", "bob", 0.5, 1) {
		t.Fatal("released target should be claimable")
	}
}

func TestClaimExpiry(t *testing.T) {
	r := testRegistry()
	r.TryClaim("t1", "alice", 1.0, 0)
	r.Expire(5)
	if r.Owner("t1") != "alice" {
		t.Fatal("claim should survive within TTL")
	}
	r.Expire(11) // age 11 > TTL 10
	if r.Owner("t1") != "" {
		t.Fatal("claim should expire after TTL")
	}
	// Expired claims can also be taken over directly.
	r.TryClaim("t2", "alice", 1.0, 0)
	if !r.TryClaim("t2", "bob", 0.1, 100) {
		t.Fatal("expired claim should be takeable by anyone")
	}
}

func TestSnapshotDeterministic(t *testing.T) {
	r := testRegistry()
	r.TryClaim("b", "alice", 1, 0)
	r.TryClaim("a", "bob", 1, 0)
	snap := r.Snapshot()
	if len(snap) != 2 || snap[0].TargetID != "a" || snap[1].TargetID != "b" {
		t.Fatalf("snapshot not sorted: %+v", snap)
	}
}
