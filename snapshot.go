package perception

// Detection is one sensor's observation of a target within a frame.
type Detection struct {
	SensorID   string
	TargetID   string
	Pos        Vec2 // noisy measured position
	Visibility VisibilityClass
	Fraction   float64
}

// FusedTarget is the merged per-frame result for one target.
type FusedTarget struct {
	TargetID   string
	Pos        Vec2
	Sensors    []string // contributing sensor IDs, sorted
	Visibility VisibilityClass
	Fraction   float64
}

// Snapshot is the explainable output of one perceiver for one tick.
type Snapshot struct {
	PerceiverID string
	Tick        int64
	Detections  []Detection
	Fused       []FusedTarget
	Tracks      []Track // valid memory tracks after this update
	Scores      []ScoreBreakdown
	Chosen      string // chosen target ID, "" if none
	Claimed     bool   // whether the claim on Chosen succeeded
}
