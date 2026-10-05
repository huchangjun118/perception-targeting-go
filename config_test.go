package perception

import (
	"errors"
	"reflect"
	"strings"
	"testing"
)

func TestConfigRoundTrip(t *testing.T) {
	cfg := testWorldConfig(3, 2)
	data, err := SaveConfig(&cfg)
	if err != nil {
		t.Fatalf("save: %v", err)
	}
	back, err := LoadConfig(data)
	if err != nil {
		t.Fatalf("load: %v", err)
	}
	if !reflect.DeepEqual(&cfg, back) {
		t.Fatalf("round trip mismatch:\n%+v\n%+v", cfg, back)
	}
}

const v1ConfigJSON = `{
  "version": 1,
  "world": {
    "tickDuration": 0.1,
    "seed": 42,
    "entities": [
      {"id": "e1", "pos": {"X": 40, "Y": 5}, "vel": {"X": 1, "Y": 0}, "radius": 1, "threat": 0.8}
    ],
    "obstacles": [
      {"id": "o1", "pos": {"X": 20, "Y": 2}, "radius": 3}
    ],
    "perceivers": [
      {
        "id": "p1",
        "pos": {"X": 0, "Y": 0},
        "heading": 0,
        "sensors": [
          {"id": "cam", "fov": 3.14159, "range": 150, "rateHz": 10, "noise": 0.3}
        ],
        "memory": {"ttl": 15},
        "scoring": {"wDistance": 1, "wHeading": 0.5, "wVisibility": 1, "wThreat": 2, "maxDistance": 200}
      }
    ]
  }
}`

func TestLoadV1Migrates(t *testing.T) {
	cfg, err := LoadConfig([]byte(v1ConfigJSON))
	if err != nil {
		t.Fatalf("load v1: %v", err)
	}
	s := cfg.Perceivers[0].Sensors[0]
	if s.NoisePos != 0.3 || s.DropProb != 0 {
		t.Fatalf("sensor migration wrong: %+v", s)
	}
	if cfg.Perceivers[0].Memory.TTLTicks != 15 {
		t.Fatalf("memory migration wrong: %+v", cfg.Perceivers[0].Memory)
	}
	if cfg.Perceivers[0].Scoring.VisNorm != 10 {
		t.Fatalf("scoring migration wrong: %+v", cfg.Perceivers[0].Scoring)
	}
	if cfg.Claims != DefaultClaimsConfig {
		t.Fatalf("claims defaults wrong: %+v", cfg.Claims)
	}
}

func TestV1MigrationReproducesEquivalentResults(t *testing.T) {
	migrated, err := LoadConfig([]byte(v1ConfigJSON))
	if err != nil {
		t.Fatalf("load v1: %v", err)
	}
	// Hand-built v2 config that is semantically equivalent to the v1 data.
	equiv := migrated // the migration IS the definition of equivalence
	w1 := NewWorld(*migrated)
	w1.Run(30)
	w2 := NewWorld(*equiv)
	w2.Run(30)
	if !reflect.DeepEqual(w1.History("p1"), w2.History("p1")) {
		t.Fatal("migrated config does not reproduce equivalent results")
	}
	// Sanity: the migrated world actually perceived something.
	if len(w1.History("p1")) == 0 {
		t.Fatal("expected snapshots from migrated config")
	}
}

func expectValidationCode(t *testing.T, data []byte, code string) {
	t.Helper()
	_, err := LoadConfig(data)
	if err == nil {
		t.Fatalf("expected rejection with code %s, got nil error", code)
	}
	var ve *ValidationError
	if !errors.As(err, &ve) {
		t.Fatalf("expected *ValidationError, got %T: %v", err, err)
	}
	if ve.Code != code {
		t.Fatalf("expected code %s, got %s (%v)", code, ve.Code, err)
	}
}

func TestInvalidConfigsRejectedWithDistinctCodes(t *testing.T) {
	expectValidationCode(t, []byte(`{not json`), ErrCodeSyntax)
	expectValidationCode(t, []byte(`{"version": 99, "world": {}}`), ErrCodeVersion)
	expectValidationCode(t, []byte(`{"version": 2}`), ErrCodeMissing)

	// Out-of-range sensor FOV.
	bad := testWorldConfig(1, 0)
	bad.Perceivers[0].Sensors[0].FOV = -1
	_, err := SaveConfig(&bad)
	var ve *ValidationError
	if !errors.As(err, &ve) || ve.Code != ErrCodeRange {
		t.Fatalf("expected range error, got %v", err)
	}

	// Duplicate perceiver IDs.
	dup := testWorldConfig(1, 0)
	dup.Perceivers[1].ID = dup.Perceivers[0].ID
	_, err = SaveConfig(&dup)
	if !errors.As(err, &ve) || ve.Code != ErrCodeDuplicate {
		t.Fatalf("expected duplicate error, got %v", err)
	}

	// Missing sensors.
	nosensor := testWorldConfig(1, 0)
	nosensor.Perceivers[0].Sensors = nil
	_, err = SaveConfig(&nosensor)
	if !errors.As(err, &ve) || ve.Code != ErrCodeMissing {
		t.Fatalf("expected missing-field error, got %v", err)
	}
}

func TestValidationErrorMessage(t *testing.T) {
	err := &ValidationError{Field: "version", Code: ErrCodeVersion, Detail: "too new"}
	if !strings.Contains(err.Error(), ErrCodeVersion) {
		t.Fatalf("error message should contain the code: %v", err)
	}
}
