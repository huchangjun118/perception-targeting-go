package perception

import "math"

// ScoringConfig holds the weights of the target scoring rule.
type ScoringConfig struct {
	WDistance   float64 `json:"wDistance"`
	WHeading    float64 `json:"wHeading"`
	WVisibility float64 `json:"wVisibility"` // sustained visible duration
	WThreat     float64 `json:"wThreat"`
	MaxDistance float64 `json:"maxDistance"` // normalization distance
	VisNorm     float64 `json:"visNorm"`     // ticks of visibility that saturate the term
}

// ScoreBreakdown makes a scoring decision explainable: every term that went
// into the total is recorded alongside it.
type ScoreBreakdown struct {
	TargetID   string
	Distance   float64 // distance term in [0,1] (1 = at the perceiver)
	Heading    float64 // heading alignment in [0,1] (1 = straight ahead)
	Visible    float64 // normalized sustained visibility in [0,1]
	Threat     float64 // intrinsic threat in [0,1]
	Confidence float64 // memory confidence multiplier in [0,1]
	Total      float64
}

func clamp01(x float64) float64 {
	if x < 0 {
		return 0
	}
	if x > 1 {
		return 1
	}
	return x
}

// ScoreTarget computes the (deterministic) score of a track for a perceiver.
//
//	total = ( wD*distance + wH*heading + wV*visible + wT*threat ) * confidence
//
// The confidence multiplier ensures decaying memories score lower than fresh
// observations, and expired tracks never reach this function at all.
func ScoreTarget(cfg ScoringConfig, perceiverPos Vec2, perceiverHeading float64, t *Track) ScoreBreakdown {
	b := ScoreBreakdown{TargetID: t.TargetID, Threat: clamp01(t.Threat), Confidence: clamp01(t.Confidence)}

	maxD := cfg.MaxDistance
	if maxD <= 0 {
		maxD = 1
	}
	b.Distance = 1 - clamp01(perceiverPos.Dist(t.Pos)/maxD)

	angTo := AngleOf(perceiverPos, t.Pos)
	b.Heading = (1 + math.Cos(AngleDiff(angTo, perceiverHeading))) / 2

	visNorm := cfg.VisNorm
	if visNorm <= 0 {
		visNorm = 1
	}
	b.Visible = clamp01(float64(t.VisibleTicks) / visNorm)

	b.Total = cfg.WDistance*b.Distance + cfg.WHeading*b.Heading +
		cfg.WVisibility*b.Visible + cfg.WThreat*b.Threat
	b.Total *= b.Confidence
	return b
}

// SelectTarget picks the best candidate. Ties on the total score are broken
// deterministically: the lexicographically smaller target ID wins. Returns
// nil if there are no candidates.
func SelectTarget(cands []ScoreBreakdown) *ScoreBreakdown {
	if len(cands) == 0 {
		return nil
	}
	best := cands[0]
	for _, c := range cands[1:] {
		if c.Total > best.Total || (c.Total == best.Total && c.TargetID < best.TargetID) {
			best = c
		}
	}
	return &best
}
