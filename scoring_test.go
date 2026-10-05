package perception

import "testing"

var testScoring = ScoringConfig{
	WDistance: 1, WHeading: 1, WVisibility: 1, WThreat: 1,
	MaxDistance: 100, VisNorm: 10,
}

func track(id string, x float64, threat float64, visTicks int64, conf float64) *Track {
	return &Track{TargetID: id, Pos: Vec2{x, 0}, Threat: threat, VisibleTicks: visTicks, Confidence: conf}
}

func TestScoreComponents(t *testing.T) {
	b := ScoreTarget(testScoring, Vec2{0, 0}, 0, track("a", 50, 0.5, 5, 1))
	if b.Distance != 0.5 {
		t.Fatalf("distance term: got %v want 0.5", b.Distance)
	}
	if b.Heading != 1 {
		t.Fatalf("heading term: got %v want 1 (straight ahead)", b.Heading)
	}
	if b.Visible != 0.5 {
		t.Fatalf("visible term: got %v want 0.5", b.Visible)
	}
	if b.Threat != 0.5 {
		t.Fatalf("threat term: got %v want 0.5", b.Threat)
	}
	want := 0.5 + 1 + 0.5 + 0.5
	if b.Total != want {
		t.Fatalf("total: got %v want %v", b.Total, want)
	}
}

func TestScoreConfidenceMultiplier(t *testing.T) {
	fresh := ScoreTarget(testScoring, Vec2{0, 0}, 0, track("a", 50, 0.5, 5, 1))
	stale := ScoreTarget(testScoring, Vec2{0, 0}, 0, track("a", 50, 0.5, 5, 0.5))
	if stale.Total != fresh.Total/2 {
		t.Fatalf("confidence not applied: fresh=%v stale=%v", fresh.Total, stale.Total)
	}
}

func TestSelectTargetTieBreakDeterministic(t *testing.T) {
	cands := []ScoreBreakdown{
		{TargetID: "b", Total: 1.5},
		{TargetID: "a", Total: 1.5},
		{TargetID: "c", Total: 1.0},
	}
	best := SelectTarget(cands)
	if best == nil || best.TargetID != "a" {
		t.Fatalf("tie must break to lexicographically smaller ID, got %+v", best)
	}
	// Order of candidates must not matter.
	rev := []ScoreBreakdown{cands[2], cands[1], cands[0]}
	if got := SelectTarget(rev); got.TargetID != "a" {
		t.Fatalf("selection depends on input order: %v", got.TargetID)
	}
	if SelectTarget(nil) != nil {
		t.Fatalf("expected nil for empty candidates")
	}
}
