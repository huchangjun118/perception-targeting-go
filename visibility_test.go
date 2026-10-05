package perception

import (
	"math"
	"testing"
)

var testSensor = SensorConfig{ID: "s1", FOV: math.Pi / 2, Range: 50, RateHz: 1}

func targetAt(x, y, r float64) *Entity {
	return &Entity{ID: "t1", Pos: Vec2{x, y}, Radius: r, Threat: 0.5}
}

func TestFullyVisible(t *testing.T) {
	rep := AssessVisibility(Vec2{0, 0}, 0, testSensor, targetAt(10, 0, 1), nil, nil)
	if rep.Class != FullyVisible {
		t.Fatalf("expected fully visible, got %v (frac=%v)", rep.Class, rep.VisibleFraction)
	}
	if rep.VisibleFraction != 1 {
		t.Fatalf("expected fraction 1, got %v", rep.VisibleFraction)
	}
}

func TestFullyOccludedByStaticObstacle(t *testing.T) {
	obs := []Obstacle{{ID: "o1", Pos: Vec2{5, 0}, Radius: 2}}
	rep := AssessVisibility(Vec2{0, 0}, 0, testSensor, targetAt(10, 0, 1), obs, nil)
	if rep.Class != NotVisible {
		t.Fatalf("expected not visible, got %v (frac=%v)", rep.Class, rep.VisibleFraction)
	}
}

func TestFullyOccludedByDynamicEntity(t *testing.T) {
	other := &Entity{ID: "t2", Pos: Vec2{5, 0}, Radius: 2}
	rep := AssessVisibility(Vec2{0, 0}, 0, testSensor, targetAt(10, 0, 1), nil, []*Entity{other, targetAt(10, 0, 1)})
	if rep.Class != NotVisible {
		t.Fatalf("expected not visible, got %v (frac=%v)", rep.Class, rep.VisibleFraction)
	}
}

func TestPartiallyVisible(t *testing.T) {
	obs := []Obstacle{{ID: "o1", Pos: Vec2{5, 0.8}, Radius: 0.5}}
	rep := AssessVisibility(Vec2{0, 0}, 0, testSensor, targetAt(10, 0, 1), obs, nil)
	if rep.Class != PartiallyVisible {
		t.Fatalf("expected partially visible, got %v (frac=%v)", rep.Class, rep.VisibleFraction)
	}
	if rep.VisibleFraction <= 0 || rep.VisibleFraction >= 1 {
		t.Fatalf("fraction out of (0,1): %v", rep.VisibleFraction)
	}
}

func TestOutOfRange(t *testing.T) {
	rep := AssessVisibility(Vec2{0, 0}, 0, testSensor, targetAt(100, 0, 1), nil, nil)
	if rep.Class != NotVisible || rep.InRange {
		t.Fatalf("expected not visible / out of range, got %+v", rep)
	}
}

func TestOutOfFOV(t *testing.T) {
	// Facing +X with 90-degree FOV; target behind at -X.
	rep := AssessVisibility(Vec2{0, 0}, 0, testSensor, targetAt(-10, 0, 1), nil, nil)
	if rep.Class != NotVisible || rep.InFOV {
		t.Fatalf("expected not visible / out of FOV, got %+v", rep)
	}
}

func TestBoundaryDeterminism(t *testing.T) {
	// Target whose near surface lies exactly at the range limit.
	tgt := targetAt(51, 0, 1) // dist - radius = 50 = Range
	rep1 := AssessVisibility(Vec2{0, 0}, 0, testSensor, tgt, nil, nil)
	if !rep1.InRange {
		t.Fatalf("expected in range at exact boundary")
	}
	// Just beyond the boundary (well above the fixed epsilon).
	tgt2 := targetAt(51.001, 0, 1)
	rep2 := AssessVisibility(Vec2{0, 0}, 0, testSensor, tgt2, nil, nil)
	if rep2.InRange {
		t.Fatalf("expected out of range just past boundary")
	}
	// Repeated evaluation is bit-identical.
	obs := []Obstacle{{ID: "o1", Pos: Vec2{5, 0.8}, Radius: 0.5}}
	a := AssessVisibility(Vec2{0, 0}, 0, testSensor, targetAt(10, 0, 1), obs, nil)
	b := AssessVisibility(Vec2{0, 0}, 0, testSensor, targetAt(10, 0, 1), obs, nil)
	if a != b {
		t.Fatalf("non-deterministic visibility: %+v vs %+v", a, b)
	}
}

func TestOccludedFractionUnion(t *testing.T) {
	// Two overlapping occluders must not be double-counted.
	f := occludedFraction(0, 0.1, [][2]float64{{0, 0.2}, {0.05, 0.2}})
	if f != 1 {
		t.Fatalf("expected full occlusion, got %v", f)
	}
	// Disjoint occluders each covering a quarter.
	f = occludedFraction(0, 0.2, [][2]float64{{-0.1, 0.05}, {0.1, 0.05}})
	if math.Abs(f-0.5) > 1e-9 {
		t.Fatalf("expected 0.5 occlusion, got %v", f)
	}
}
