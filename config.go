package perception

import (
	"encoding/json"
	"fmt"
	"math"
)

// ConfigVersion is the current serialization version.
const ConfigVersion = 2

// Validation error codes — distinguishable rejection reasons.
const (
	ErrCodeMissing   = "missing_field"
	ErrCodeRange     = "out_of_range"
	ErrCodeDuplicate = "duplicate_id"
	ErrCodeVersion   = "unsupported_version"
	ErrCodeSyntax    = "malformed_data"
)

// ValidationError describes why a config was rejected.
type ValidationError struct {
	Field  string
	Code   string
	Detail string
}

func (e *ValidationError) Error() string {
	return fmt.Sprintf("config invalid: %s (%s): %s", e.Field, e.Code, e.Detail)
}

// envelope is the on-disk format: a version tag plus the world config.
type envelope struct {
	Version int             `json:"version"`
	World   json.RawMessage `json:"world"`
}

// SaveConfig serializes a config to JSON at the current version.
func SaveConfig(cfg *WorldConfig) ([]byte, error) {
	if err := ValidateConfig(cfg); err != nil {
		return nil, err
	}
	raw, err := json.Marshal(cfg)
	if err != nil {
		return nil, err
	}
	return json.Marshal(envelope{Version: ConfigVersion, World: raw})
}

// LoadConfig parses JSON of any supported version, migrates it to the current
// version and validates it. Invalid or incomplete data is rejected with a
// distinguishable *ValidationError.
func LoadConfig(data []byte) (*WorldConfig, error) {
	var env envelope
	if err := json.Unmarshal(data, &env); err != nil {
		return nil, &ValidationError{Field: "", Code: ErrCodeSyntax, Detail: err.Error()}
	}
	if env.Version < 1 || env.Version > ConfigVersion {
		return nil, &ValidationError{
			Field:  "version",
			Code:   ErrCodeVersion,
			Detail: fmt.Sprintf("supported versions: 1..%d, got %d", ConfigVersion, env.Version),
		}
	}
	if len(env.World) == 0 {
		return nil, &ValidationError{Field: "world", Code: ErrCodeMissing, Detail: "missing world config"}
	}
	var cfg WorldConfig
	switch env.Version {
	case 1:
		migrated, err := migrateV1(env.World)
		if err != nil {
			return nil, err
		}
		cfg = *migrated
	default: // version 2
		if err := json.Unmarshal(env.World, &cfg); err != nil {
			return nil, &ValidationError{Field: "world", Code: ErrCodeSyntax, Detail: err.Error()}
		}
	}
	if err := ValidateConfig(&cfg); err != nil {
		return nil, err
	}
	return &cfg, nil
}

// --- v1 schema and migration ---
//
// Differences between v1 and v2:
//   - v1 sensors have a single "noise" field; v2 splits it into noisePos and
//     dropProb (dropProb defaults to 0, preserving v1 behavior).
//   - v1 memory uses "ttl" instead of "ttlTicks".
//   - v1 has no claims config; migration fills in defaults.
//   - v1 scoring lacks visNorm; migration defaults it to 10 ticks.

type sensorV1 struct {
	ID     string  `json:"id"`
	FOV    float64 `json:"fov"`
	Range  float64 `json:"range"`
	RateHz float64 `json:"rateHz"`
	Noise  float64 `json:"noise"`
}

type memoryV1 struct {
	TTL int64 `json:"ttl"`
}

type scoringV1 struct {
	WDistance   float64 `json:"wDistance"`
	WHeading    float64 `json:"wHeading"`
	WVisibility float64 `json:"wVisibility"`
	WThreat     float64 `json:"wThreat"`
	MaxDistance float64 `json:"maxDistance"`
}

type perceiverV1 struct {
	ID      string     `json:"id"`
	Pos     Vec2       `json:"pos"`
	Heading float64    `json:"heading"`
	Sensors []sensorV1 `json:"sensors"`
	Memory  memoryV1   `json:"memory"`
	Scoring scoringV1  `json:"scoring"`
}

type worldV1 struct {
	TickDuration float64       `json:"tickDuration"`
	Seed         int64         `json:"seed"`
	Entities     []Entity      `json:"entities"`
	Obstacles    []Obstacle    `json:"obstacles"`
	Perceivers   []perceiverV1 `json:"perceivers"`
	Workers      int           `json:"workers"`
}

// DefaultClaimsConfig is used when migrating v1 configs that predate claims.
var DefaultClaimsConfig = ClaimsConfig{PreemptMargin: 0.05, HoldTicks: 5, TTLTicks: 20}

func migrateV1(raw json.RawMessage) (*WorldConfig, error) {
	var old worldV1
	if err := json.Unmarshal(raw, &old); err != nil {
		return nil, &ValidationError{Field: "world", Code: ErrCodeSyntax, Detail: err.Error()}
	}
	cfg := &WorldConfig{
		TickDuration: old.TickDuration,
		Seed:         old.Seed,
		Entities:     old.Entities,
		Obstacles:    old.Obstacles,
		Claims:       DefaultClaimsConfig,
		Workers:      old.Workers,
	}
	for _, p := range old.Perceivers {
		pc := PerceiverConfig{
			ID:      p.ID,
			Pos:     p.Pos,
			Heading: p.Heading,
			Memory:  MemoryConfig{TTLTicks: p.Memory.TTL},
			Scoring: ScoringConfig{
				WDistance:   p.Scoring.WDistance,
				WHeading:    p.Scoring.WHeading,
				WVisibility: p.Scoring.WVisibility,
				WThreat:     p.Scoring.WThreat,
				MaxDistance: p.Scoring.MaxDistance,
				VisNorm:     10,
			},
		}
		for _, s := range p.Sensors {
			pc.Sensors = append(pc.Sensors, SensorConfig{
				ID:       s.ID,
				FOV:      s.FOV,
				Range:    s.Range,
				RateHz:   s.RateHz,
				NoisePos: s.Noise,
				DropProb: 0,
			})
		}
		cfg.Perceivers = append(cfg.Perceivers, pc)
	}
	return cfg, nil
}

// --- validation ---

func checkUnique(field string, ids []string) error {
	seen := map[string]bool{}
	for _, id := range ids {
		if id == "" {
			return &ValidationError{Field: field, Code: ErrCodeMissing, Detail: "empty id"}
		}
		if seen[id] {
			return &ValidationError{Field: field, Code: ErrCodeDuplicate, Detail: "duplicate id " + id}
		}
		seen[id] = true
	}
	return nil
}

// ValidateConfig checks a config and returns the first problem found, or nil.
func ValidateConfig(cfg *WorldConfig) error {
	if cfg == nil {
		return &ValidationError{Field: "", Code: ErrCodeMissing, Detail: "nil config"}
	}
	if cfg.TickDuration <= 0 {
		return &ValidationError{Field: "tickDuration", Code: ErrCodeRange, Detail: "must be > 0"}
	}
	if cfg.Workers < 0 {
		return &ValidationError{Field: "workers", Code: ErrCodeRange, Detail: "must be >= 0"}
	}

	entIDs := make([]string, len(cfg.Entities))
	for i, e := range cfg.Entities {
		entIDs[i] = e.ID
		if e.Radius <= 0 {
			return &ValidationError{Field: "entities[" + e.ID + "].radius", Code: ErrCodeRange, Detail: "must be > 0"}
		}
		if e.Threat < 0 || e.Threat > 1 {
			return &ValidationError{Field: "entities[" + e.ID + "].threat", Code: ErrCodeRange, Detail: "must be in [0,1]"}
		}
	}
	if err := checkUnique("entities", entIDs); err != nil {
		return err
	}

	obIDs := make([]string, len(cfg.Obstacles))
	for i, o := range cfg.Obstacles {
		obIDs[i] = o.ID
		if o.Radius <= 0 {
			return &ValidationError{Field: "obstacles[" + o.ID + "].radius", Code: ErrCodeRange, Detail: "must be > 0"}
		}
	}
	if err := checkUnique("obstacles", obIDs); err != nil {
		return err
	}

	if cfg.Claims.PreemptMargin < 0 || cfg.Claims.HoldTicks < 0 || cfg.Claims.TTLTicks < 0 {
		return &ValidationError{Field: "claims", Code: ErrCodeRange, Detail: "margins and tick counts must be >= 0"}
	}

	percIDs := make([]string, len(cfg.Perceivers))
	for i, p := range cfg.Perceivers {
		percIDs[i] = p.ID
		if len(p.Sensors) == 0 {
			return &ValidationError{Field: "perceivers[" + p.ID + "].sensors", Code: ErrCodeMissing, Detail: "at least one sensor required"}
		}
		sensorIDs := make([]string, len(p.Sensors))
		for j, s := range p.Sensors {
			sensorIDs[j] = s.ID
			if s.FOV <= 0 || s.FOV > 2*math.Pi {
				return &ValidationError{Field: fmt.Sprintf("perceivers[%s].sensors[%s].fov", p.ID, s.ID), Code: ErrCodeRange, Detail: "must be in (0, 2pi]"}
			}
			if s.Range <= 0 {
				return &ValidationError{Field: fmt.Sprintf("perceivers[%s].sensors[%s].range", p.ID, s.ID), Code: ErrCodeRange, Detail: "must be > 0"}
			}
			if s.RateHz <= 0 {
				return &ValidationError{Field: fmt.Sprintf("perceivers[%s].sensors[%s].rateHz", p.ID, s.ID), Code: ErrCodeRange, Detail: "must be > 0"}
			}
			if s.NoisePos < 0 {
				return &ValidationError{Field: fmt.Sprintf("perceivers[%s].sensors[%s].noisePos", p.ID, s.ID), Code: ErrCodeRange, Detail: "must be >= 0"}
			}
			if s.DropProb < 0 || s.DropProb >= 1 {
				return &ValidationError{Field: fmt.Sprintf("perceivers[%s].sensors[%s].dropProb", p.ID, s.ID), Code: ErrCodeRange, Detail: "must be in [0,1)"}
			}
		}
		if err := checkUnique("perceivers["+p.ID+"].sensors", sensorIDs); err != nil {
			return err
		}
		if p.Memory.TTLTicks < 1 {
			return &ValidationError{Field: "perceivers[" + p.ID + "].memory.ttlTicks", Code: ErrCodeRange, Detail: "must be >= 1"}
		}
		sc := p.Scoring
		if sc.WDistance < 0 || sc.WHeading < 0 || sc.WVisibility < 0 || sc.WThreat < 0 {
			return &ValidationError{Field: "perceivers[" + p.ID + "].scoring", Code: ErrCodeRange, Detail: "weights must be >= 0"}
		}
		if sc.MaxDistance <= 0 {
			return &ValidationError{Field: "perceivers[" + p.ID + "].scoring.maxDistance", Code: ErrCodeRange, Detail: "must be > 0"}
		}
		if sc.VisNorm <= 0 {
			return &ValidationError{Field: "perceivers[" + p.ID + "].scoring.visNorm", Code: ErrCodeRange, Detail: "must be > 0"}
		}
	}
	return checkUnique("perceivers", percIDs)
}
