// Package sim 提供场景、感知者代理与确定性并发推进引擎。
package sim

import (
	"github.com/huchangjun118/perception-targeting-go/geom"
	"github.com/huchangjun118/perception-targeting-go/perception"
	"github.com/huchangjun118/perception-targeting-go/targeting"
)

// Agent 感知者代理。
type Agent struct {
	ID       string                    `json:"id"`
	Pos      geom.Vec2                 `json:"pos"`
	Heading  float64                   `json:"heading"` // 朝向（弧度）
	Sensors  []perception.SensorConfig `json:"sensors"`
	Weights  targeting.ScoreWeights    `json:"weights"`
	MemCfg   perception.MemoryConfig   `json:"memory"`
	MaxRange float64                   `json:"max_range"` // 评分归一化用的最大距离
}

// Scene 完整场景定义。
type Scene struct {
	StepHz    float64             `json:"step_hz"` // 全局推进步频
	Obstacles []geom.Segment      `json:"obstacles"`
	Targets   []perception.Target `json:"targets"`
	Agents    []Agent             `json:"agents"`
}

// FrameResult 一帧的确定性输出。
type FrameResult struct {
	Tick      uint64
	Snapshots []perception.Snapshot // 按 AgentID 排序
	Decisions []targeting.Decision  // 按 AgentID 排序
	Claims    map[string]string     // targetID -> agentID
}
