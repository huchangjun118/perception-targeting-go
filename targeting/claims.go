package targeting

import "sort"

// claimEps 抢占判定的分数容差。
const claimEps = 1e-9

// ClaimRegistry 目标声明注册表：同一目标同一时刻只能被一个代理占用。
// 所有方法由 sim 引擎在串行决策阶段调用，无需内部锁。
type ClaimRegistry struct {
	owners map[string]string // targetID -> agentID
}

func NewClaimRegistry() *ClaimRegistry {
	return &ClaimRegistry{owners: make(map[string]string)}
}

// Owner 返回目标当前占用者。
func (r *ClaimRegistry) Owner(targetID string) (string, bool) {
	a, ok := r.owners[targetID]
	return a, ok
}

// Claim 尝试声明目标。规则确定：
//   - 空闲或由自己占用 → 成功；
//   - 被他人占用：挑战者分数 > 占用者分数 + eps 时抢占；
//     分数差在 eps 内视为平局，agentID 字典序小者优先。
//
// scoreOf 返回指定代理对该目标的当前评分。
func (r *ClaimRegistry) Claim(agentID, targetID string, score float64,
	scoreOf func(agentID string) float64) bool {
	owner, ok := r.owners[targetID]
	if !ok || owner == agentID {
		r.owners[targetID] = agentID
		return true
	}
	own := scoreOf(owner)
	if score > own+claimEps ||
		(score > own-claimEps && agentID < owner) {
		r.owners[targetID] = agentID
		return true
	}
	return false
}

// Release 释放目标（仅占用者可释放）。
func (r *ClaimRegistry) Release(agentID, targetID string) bool {
	if r.owners[targetID] != agentID {
		return false
	}
	delete(r.owners, targetID)
	return true
}

// ClaimsOf 返回某代理当前占用的所有目标（有序）。
func (r *ClaimRegistry) ClaimsOf(agentID string) []string {
	var out []string
	for t, a := range r.owners {
		if a == agentID {
			out = append(out, t)
		}
	}
	sort.Strings(out)
	return out
}

// Snapshot 返回 targetID->agentID 的副本，用于输出确定性。
func (r *ClaimRegistry) Snapshot() map[string]string {
	out := make(map[string]string, len(r.owners))
	for k, v := range r.owners {
		out[k] = v
	}
	return out
}
