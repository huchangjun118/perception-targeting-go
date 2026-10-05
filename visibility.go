package perception

import "math"

// VisibilityReport describes how well a target is visible from an observer.
type VisibilityReport struct {
	Class           VisibilityClass
	VisibleFraction float64 // fraction of the target's angular extent effectively visible, in [0,1]
	InRange         bool
	InFOV           bool
	Distance        float64
}

// fovFraction computes the fraction of the angular interval
// [center-half, center+half] that lies inside the FOV cone
// [facing-fov/2, facing+fov/2].
func fovFraction(center, half, facing, fov float64) float64 {
	if half <= 0 {
		return 0
	}
	if fov >= 2*math.Pi {
		return 1
	}
	rel := AngleDiff(center, facing) // target center relative to facing
	lo, hi := -half-rel, half-rel    // target interval in facing-relative coords
	flo, fhi := -fov/2, fov/2
	a := math.Max(lo, flo)
	b := math.Min(hi, fhi)
	if b <= a+boundaryEpsilon {
		return 0
	}
	f := (b - a) / (2 * half)
	if f > 1 {
		return 1
	}
	return f
}

// AssessVisibility computes the visibility of a target entity from an observer
// at obsPos with the given facing and sensor parameters, considering both
// static obstacles and other dynamic entities as occluders.
//
// The decision is deterministic: identical inputs always produce identical
// results; boundary cases use a fixed epsilon.
func AssessVisibility(obsPos Vec2, facing float64, sensor SensorConfig, target *Entity, obstacles []Obstacle, others []*Entity) VisibilityReport {
	rep := VisibilityReport{}
	dist := obsPos.Dist(target.Pos)
	rep.Distance = dist

	// Range: the nearest surface of the target must be within range.
	rep.InRange = dist-target.Radius <= sensor.Range+boundaryEpsilon
	if !rep.InRange {
		rep.Class = NotVisible
		return rep
	}

	ang := AngleOf(obsPos, target.Pos)
	halfT := angularIntervalOfCircle(dist, target.Radius)

	// FOV coverage of the target's angular extent.
	ff := fovFraction(ang, halfT, facing, sensor.FOV)
	rep.InFOV = ff > boundaryEpsilon
	if !rep.InFOV {
		rep.Class = NotVisible
		return rep
	}

	// Collect occluder angular intervals (only occluders nearer than the
	// target center can block the line of sight).
	occluders := make([][2]float64, 0, len(obstacles)+len(others))
	addOccluder := func(pos Vec2, radius float64) {
		d := obsPos.Dist(pos)
		if d >= dist-boundaryEpsilon {
			return // behind or at the target: cannot occlude
		}
		if d <= radius+boundaryEpsilon {
			// Observer is inside the occluder: everything is blocked.
			occluders = append(occluders, [2]float64{ang, math.Pi})
			return
		}
		occluders = append(occluders, [2]float64{AngleOf(obsPos, pos), angularIntervalOfCircle(d, radius)})
	}
	for _, ob := range obstacles {
		addOccluder(ob.Pos, ob.Radius)
	}
	for _, e := range others {
		if e.ID == target.ID {
			continue
		}
		addOccluder(e.Pos, e.Radius)
	}

	occ := occludedFraction(ang, halfT, occluders)
	vis := ff * (1 - occ)
	if vis < 0 {
		vis = 0
	}
	if vis > 1 {
		vis = 1
	}
	rep.VisibleFraction = vis
	switch {
	case vis <= boundaryEpsilon:
		rep.Class = NotVisible
	case vis >= 1-boundaryEpsilon:
		rep.Class = FullyVisible
	default:
		rep.Class = PartiallyVisible
	}
	return rep
}
