package perception

import "math"

// Vec2 is a 2D vector / point.
type Vec2 struct {
	X, Y float64
}

func (a Vec2) Add(b Vec2) Vec2      { return Vec2{a.X + b.X, a.Y + b.Y} }
func (a Vec2) Sub(b Vec2) Vec2      { return Vec2{a.X - b.X, a.Y - b.Y} }
func (a Vec2) Scale(s float64) Vec2 { return Vec2{a.X * s, a.Y * s} }
func (a Vec2) Len() float64         { return math.Hypot(a.X, a.Y) }
func (a Vec2) Dist(b Vec2) float64  { return a.Sub(b).Len() }

// AngleOf returns the absolute angle of the vector (to - from).
func AngleOf(from, to Vec2) float64 {
	d := to.Sub(from)
	return math.Atan2(d.Y, d.X)
}

// NormalizeAngle wraps an angle into (-pi, pi].
func NormalizeAngle(a float64) float64 {
	for a <= -math.Pi {
		a += 2 * math.Pi
	}
	for a > math.Pi {
		a -= 2 * math.Pi
	}
	return a
}

// AngleDiff returns the signed smallest difference a-b in (-pi, pi].
func AngleDiff(a, b float64) float64 { return NormalizeAngle(a - b) }

// boundaryEpsilon is the fixed epsilon used for all boundary decisions so
// that visibility on the exact edge of an occluder / FOV is deterministic.
const boundaryEpsilon = 1e-9

// angularIntervalOfCircle returns the angular half-extent of a circle of the
// given radius at the given distance (center to observer). If the observer is
// inside or on the circle it returns pi (covers everything).
func angularIntervalOfCircle(dist, radius float64) float64 {
	if dist <= radius+boundaryEpsilon {
		return math.Pi
	}
	return math.Asin(radius / dist)
}

// occludedFraction computes the fraction of the angular interval
// [center-half, center+half] that is covered by the union of the occluder
// intervals (each given as {center, half} relative to the same observer).
// All intervals are normalized to be centered near the target center so the
// wrap-around at +-pi is handled deterministically.
func occludedFraction(center, half float64, occluders [][2]float64) float64 {
	if half <= 0 {
		return 0
	}
	if half >= math.Pi {
		half = math.Pi
	}
	// Work in coordinates relative to the target center to avoid wrap issues.
	lo, hi := -half, half
	// Clamp occluders into [lo, hi] and merge.
	type seg struct{ a, b float64 }
	segs := make([]seg, 0, len(occluders))
	for _, oc := range occluders {
		rel := AngleDiff(oc[0], center) // occluder center relative to target center
		a, b := rel-oc[1], rel+oc[1]
		if b < lo+boundaryEpsilon || a > hi-boundaryEpsilon {
			continue
		}
		if a < lo {
			a = lo
		}
		if b > hi {
			b = hi
		}
		segs = append(segs, seg{a, b})
	}
	if len(segs) == 0 {
		return 0
	}
	// Sort by start (insertion sort; few occluders, fully deterministic).
	for i := 1; i < len(segs); i++ {
		for j := i; j > 0 && segs[j].a < segs[j-1].a; j-- {
			segs[j], segs[j-1] = segs[j-1], segs[j]
		}
	}
	covered := 0.0
	curA, curB := segs[0].a, segs[0].b
	for _, s := range segs[1:] {
		if s.a > curB+boundaryEpsilon {
			covered += curB - curA
			curA, curB = s.a, s.b
		} else if s.b > curB {
			curB = s.b
		}
	}
	covered += curB - curA
	total := hi - lo
	if total <= 0 {
		return 0
	}
	f := covered / total
	if f < 0 {
		return 0
	}
	if f > 1 {
		return 1
	}
	return f
}
