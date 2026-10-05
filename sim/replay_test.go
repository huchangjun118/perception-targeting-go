package sim

import (
	"testing"
)

// BenchmarkStepLarge 大规模场景推进性能基准。
func BenchmarkStepLarge(b *testing.B) {
	scene := testScene()
	for i := 0; i < 50; i++ {
		a := scene.Agents[0]
		a.ID = a.ID + "-dup" + string(rune('A'+i%26)) + string(rune('0'+i/26))
		scene.Agents = append(scene.Agents, a)
	}
	e := NewEngine(scene, 1, 8)
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		e.Step()
	}
}
