package targeting

import (
	"testing"

	"github.com/huchangjun118/perception-targeting-go/geom"
	"github.com/huchangjun118/perception-targeting-go/perception"
)

func entry(id string, threat float64) perception.MemoryEntry {
	return perception.MemoryEntry{TargetID: id, Threat: threat, Confidence: 1}
}

func TestScoreComponents(t *testing.T) {
	w := ScoreWeights{Distance: 1, Heading: 1, Persist: 1, Threat: 1, Confidence: 1}
	near := Candidate{Entry: entry("a", 1), Distance: 0, Bearing: 0, PersistTick: 1000}
	far := Candidate{Entry: entry("b", 0), Distance: 90, Bearing: 3.0, PersistTick: 0}
	if Score(near, w, 100) <= Score(far, w, 100) {
		t.Fatalf("near/aligned/threatening should outscore far: %v vs %v",
			Score(near, w, 100), Score(far, w, 100))
	}
	// 单项验证：距离权重独占时，距离近者胜
	wd := ScoreWeights{Distance: 1}
	if Score(far, wd, 100) >= Score(near, wd, 100) {
		t.Fatal("distance-only weighting broken")
	}
}

func TestSelectTieBreakDeterministic(t *testing.T) {
	w := ScoreWeights{Threat: 1}
	c1 := Candidate{Entry: entry("t-b", 0.5)}
	c2 := Candidate{Entry: entry("t-a", 0.5)}
	// 两种输入顺序都必须选出 t-a
	for _, cands := range [][]Candidate{{c1, c2}, {c2, c1}} {
		d := Select("p", cands, w, 100)
		if d.TargetID != "t-a" {
			t.Fatalf("tie must break to lexicographically smaller id, got %q", d.TargetID)
		}
		if d.Reason == "" {
			t.Fatal("decision must carry an explanation")
		}
	}
}

func TestSelectEmpty(t *testing.T) {
	d := Select("p", nil, ScoreWeights{}, 100)
	if d.TargetID != "" {
		t.Fatal("expected empty target for no candidates")
	}
}

func TestClaimExclusive(t *testing.T) {
	r := NewClaimRegistry()
	if !r.Claim("a1", "t1", 1.0, nil) {
		t.Fatal("first claim should succeed")
	}
	// 他人以更低分抢占失败
	if r.Claim("a2", "t1", 0.5, func(string) float64 { return 1.0 }) {
		t.Fatal("lower score must not preempt")
	}
	if owner, _ := r.Owner("t1"); owner != "a1" {
		t.Fatalf("owner should remain a1, got %q", owner)
	}
	// 更高分抢占成功
	if !r.Claim("a2", "t1", 2.0, func(string) float64 { return 1.0 }) {
		t.Fatal("higher score should preempt")
	}
	if owner, _ := r.Owner("t1"); owner != "a2" {
		t.Fatalf("owner should be a2, got %q", owner)
	}
}

func TestClaimTieBreakByAgentID(t *testing.T) {
	r := NewClaimRegistry()
	r.Claim("b-agent", "t1", 1.0, nil)
	// 平分：字典序更小者胜
	if !r.Claim("a-agent", "t1", 1.0, func(string) float64 { return 1.0 }) {
		t.Fatal("tie should favor lexicographically smaller agent id")
	}
	// 反向不成立
	r2 := NewClaimRegistry()
	r2.Claim("a-agent", "t1", 1.0, nil)
	if r2.Claim("b-agent", "t1", 1.0, func(string) float64 { return 1.0 }) {
		t.Fatal("tie must not favor larger agent id")
	}
}

func TestRelease(t *testing.T) {
	r := NewClaimRegistry()
	r.Claim("a1", "t1", 1.0, nil)
	if r.Release("a2", "t1") {
		t.Fatal("non-owner must not release")
	}
	if !r.Release("a1", "t1") {
		t.Fatal("owner release should succeed")
	}
	if _, ok := r.Owner("t1"); ok {
		t.Fatal("target should be free after release")
	}
}

func TestClaimsOfSorted(t *testing.T) {
	r := NewClaimRegistry()
	r.Claim("a1", "t2", 1, nil)
	r.Claim("a1", "t1", 1, nil)
	got := r.ClaimsOf("a1")
	if len(got) != 2 || got[0] != "t1" || got[1] != "t2" {
		t.Fatalf("claims must be sorted, got %v", got)
	}
}

var _ = geom.Vec2{} // 保留 geom 引用以防后续用例需要
