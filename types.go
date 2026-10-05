package perception

// Entity is a dynamic object in the scene (a potential target).
type Entity struct {
	ID      string  `json:"id"`
	Pos     Vec2    `json:"pos"`
	Vel     Vec2    `json:"vel"`
	Radius  float64 `json:"radius"`
	Threat  float64 `json:"threat"` // intrinsic threat level in [0,1]
	Heading float64 `json:"heading"`
}

// Obstacle is a static occluder.
type Obstacle struct {
	ID     string  `json:"id"`
	Pos    Vec2    `json:"pos"`
	Radius float64 `json:"radius"`
}

// SensorConfig describes one sensor mounted on a perceiver.
type SensorConfig struct {
	ID       string  `json:"id"`
	FOV      float64 `json:"fov"`      // full field-of-view angle in radians
	Range    float64 `json:"range"`    // max detection range
	RateHz   float64 `json:"rateHz"`   // update frequency relative to world tick duration
	NoisePos float64 `json:"noisePos"` // gaussian position noise stddev (meters)
	DropProb float64 `json:"dropProb"` // probability of dropping a detection [0,1)
}

// VisibilityClass distinguishes fully visible / partially visible / invisible.
type VisibilityClass int

const (
	NotVisible VisibilityClass = iota
	PartiallyVisible
	FullyVisible
)

func (v VisibilityClass) String() string {
	switch v {
	case FullyVisible:
		return "full"
	case PartiallyVisible:
		return "partial"
	default:
		return "none"
	}
}
