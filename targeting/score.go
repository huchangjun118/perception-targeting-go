// Package targeting 提供威胁评分与目标选择。
package targeting

import (
	"fmt"
	"math"

	"github.com/huchangjun118/perception-targeting-go/perception"
)

// ScoreWeights 评分权重配置（均可配置）。
type ScoreWeights struct {
	Distance   float64 `json:"distance"`   // 距离越近分越高
	Heading    float64 `json:"heading"`    // 越朝向感知者分越高
	Persist    float64 `json:"persist"`    // 持续可见时长越长分越高
	Threat     float64 `json:"threat"`     // 固有威胁度
	Confidence float64 `json:"confidence"` // 感知置信度
}

// Candidate 参与评分的目标候选。
type Candidate struct {
	Entry       perception.MemoryEntry
	Distance    float64 // 与感知者的距离
	Bearing     float64 // 相对感知者朝向的方位角（弧度）
	PersistTick uint64  // 持续可见时长
}

// clamp01 将 v 截断到 [0,1]。
func clamp01(v float64) float64 {
	if v < 0 {
		return 0
	}
	if v > 1 {
		return 1
	}
	return v
}

// Score 计算单个候选的加权总分（各分项归一化到 [0,1]）。
// 分项规则：
//
//	距离   1 - d/maxRange
//	朝向   1 - |bearing|/pi
//	持续   persist/(persist+10)，平滑饱和
//	威胁   目标固有威胁度
//	置信   记忆置信度
func Score(c Candidate, w ScoreWeights, maxRange float64) float64 {
	var distScore float64
	if maxRange > 0 {
		distScore = clamp01(1 - c.Distance/maxRange)
	}
	headScore := clamp01(1 - math.Abs(c.Bearing)/math.Pi)
	persistScore := float64(c.PersistTick) / (float64(c.PersistTick) + 10)
	return w.Distance*distScore +
		w.Heading*headScore +
		w.Threat*clamp01(c.Entry.Threat) +
		w.Persist*persistScore +
		w.Confidence*clamp01(c.Entry.Confidence)
}

// Decision 一次目标选择的可解释结果。
type Decision struct {
	PerceiverID string
	TargetID    string // 空表示无可用目标
	Score       float64
	Reason      string // 可解释说明（含平局裁决依据）
}

// scoreEps 平局判定容差：分差小于它视为平局。
const scoreEps = 1e-9

// Select 从候选中选出最高分目标。
// 平局裁决确定：分数相同（差 < 1e-9）按 TargetID 字典序取小者。
func Select(perceiverID string, cands []Candidate, w ScoreWeights, maxRange float64) Decision {
	if len(cands) == 0 {
		return Decision{PerceiverID: perceiverID, Reason: "no candidates"}
	}
	best := -math.MaxFloat64
	bestID := ""
	tie := false
	for _, c := range cands {
		s := Score(c, w, maxRange)
		if s > best+scoreEps {
			best, bestID, tie = s, c.Entry.TargetID, false
		} else if math.Abs(s-best) <= scoreEps && c.Entry.TargetID < bestID {
			bestID, tie = c.Entry.TargetID, true
		} else if math.Abs(s-best) <= scoreEps {
			tie = true
		}
	}
	reason := fmt.Sprintf("best score %.6f among %d candidates", best, len(cands))
	if tie {
		reason += "; tie broken by lexicographically smallest target id"
	}
	return Decision{PerceiverID: perceiverID, TargetID: bestID, Score: best, Reason: reason}
}
