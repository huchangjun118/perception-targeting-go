package sim

import (
	"math/rand"
	"sort"
	"sync"

	"github.com/huchangjun118/perception-targeting-go/geom"
	"github.com/huchangjun118/perception-targeting-go/perception"
	"github.com/huchangjun118/perception-targeting-go/targeting"
)

// rngAdapter 把 *rand.Rand 适配为 perception.RandSource。
type rngAdapter struct{ r *rand.Rand }

func (a rngAdapter) Float64() float64     { return a.r.Float64() }
func (a rngAdapter) NormFloat64() float64 { return a.r.NormFloat64() }

// agentState 每个代理的运行时状态。
type agentState struct {
	mem     *perception.Memory
	rng     rngAdapter
	persist map[string]uint64 // targetID -> 连续可见 tick 数
}

// Engine 确定性仿真引擎。
// 推进分两个阶段：感知阶段可并发（只读世界状态，结果写入各自槽位），
// 决策与声明阶段按 AgentID 排序串行执行，保证结果与调度无关。
type Engine struct {
	scene   Scene
	tick    uint64
	seed    int64
	workers int

	states []agentState // 与 scene.Agents 同序
	claims *targeting.ClaimRegistry
	world  perception.WorldView
}

// NewEngine 创建引擎。seed 固定时整个推进序列可复现。
// 场景会被深拷贝，引擎推进不影响调用方持有的数据。
func NewEngine(scene Scene, seed int64, workers int) *Engine {
	if workers < 1 {
		workers = 1
	}
	// 深拷贝切片，避免推进时修改调用方数据。
	scene.Obstacles = append([]geom.Segment(nil), scene.Obstacles...)
	scene.Targets = append([]perception.Target(nil), scene.Targets...)
	scene.Agents = append([]Agent(nil), scene.Agents...)
	for i := range scene.Agents {
		scene.Agents[i].Sensors = append([]perception.SensorConfig(nil), scene.Agents[i].Sensors...)
	}
	e := &Engine{
		scene:   scene,
		seed:    seed,
		workers: workers,
		claims:  targeting.NewClaimRegistry(),
		world: perception.WorldView{
			Obstacles: scene.Obstacles,
			Targets:   scene.Targets,
		},
	}
	// 代理按 ID 排序，保证遍历顺序确定。
	sort.Slice(e.scene.Agents, func(i, j int) bool { return e.scene.Agents[i].ID < e.scene.Agents[j].ID })
	e.states = make([]agentState, len(e.scene.Agents))
	for i, a := range e.scene.Agents {
		e.states[i] = agentState{
			mem:     perception.NewMemory(a.MemCfg),
			rng:     rngAdapter{rand.New(rand.NewSource(seed + int64(i)*7919))},
			persist: make(map[string]uint64),
		}
	}
	return e
}

// Tick 返回当前 tick。
func (e *Engine) Tick() uint64 { return e.tick }

// Claims 返回当前声明注册表（只读副本）。
func (e *Engine) Claims() *targeting.ClaimRegistry { return e.claims }

// Step 推进一帧，返回确定性结果。
func (e *Engine) Step() FrameResult {
	tick := e.tick
	n := len(e.scene.Agents)
	snaps := make([]perception.Snapshot, n)

	// 阶段一：感知（并发，槽位隔离，结果与调度无关）。
	sem := make(chan struct{}, e.workers)
	var wg sync.WaitGroup
	for i := range e.scene.Agents {
		wg.Add(1)
		go func(i int) {
			defer wg.Done()
			sem <- struct{}{}
			defer func() { <-sem }()
			snaps[i] = e.perceive(i, tick)
		}(i)
	}
	wg.Wait()

	// 阶段二：记忆更新 + 决策 + 声明（按 AgentID 序串行）。
	threats := make(map[string]float64, len(e.world.Targets))
	for _, t := range e.world.Targets {
		threats[t.ID] = t.Threat
	}
	res := FrameResult{Tick: tick}
	for i, a := range e.scene.Agents {
		st := &e.states[i]
		st.mem.Update(snaps[i], threats)
		// 持续可见计数：本帧观测到则 +1，否则清零。
		for id := range st.persist {
			if _, ok := snaps[i].Find(id); !ok {
				delete(st.persist, id)
			}
		}
		for _, p := range snaps[i].Targets {
			st.persist[p.TargetID]++
		}

		dec := e.decide(a, st, tick)
		res.Decisions = append(res.Decisions, dec)
		res.Snapshots = append(res.Snapshots, snaps[i])

		// 声明/释放：每个代理至多持有一个目标。
		prev := e.claims.ClaimsOf(a.ID)
		if dec.TargetID == "" {
			for _, t := range prev {
				e.claims.Release(a.ID, t)
			}
		} else {
			for _, t := range prev {
				if t != dec.TargetID {
					e.claims.Release(a.ID, t)
				}
			}
			e.claims.Claim(a.ID, dec.TargetID, dec.Score, func(agentID string) float64 {
				// 占用者本帧对该目标的评分；未评分（未选择它）则为 0。
				for _, d := range res.Decisions {
					if d.PerceiverID == agentID && d.TargetID == dec.TargetID {
						return d.Score
					}
				}
				return 0
			})
		}
	}
	res.Claims = e.claims.Snapshot()

	// 阶段三：世界推进（确定性运动）。
	for i := range e.scene.Targets {
		e.scene.Targets[i].Pos = e.scene.Targets[i].Pos.Add(e.scene.Targets[i].Velocity)
	}
	e.world.Targets = e.scene.Targets
	e.tick++
	return res
}

// perceive 计算单个代理的融合快照。
func (e *Engine) perceive(i int, tick uint64) perception.Snapshot {
	a := e.scene.Agents[i]
	st := &e.states[i]
	var obs []perception.Observation
	for _, sc := range a.Sensors {
		s := perception.Sensor{Cfg: sc}
		obs = append(obs, s.Scan(tick, e.scene.StepHz, a.ID, a.Pos, a.Heading, e.world, st.rng)...)
	}
	return perception.Fuse(a.ID, tick, obs)
}

// decide 为单个代理构建候选并选择目标。
func (e *Engine) decide(a Agent, st *agentState, tick uint64) targeting.Decision {
	entries := st.mem.Valid(tick)
	cands := make([]targeting.Candidate, 0, len(entries))
	for _, en := range entries {
		to := en.Position.Sub(a.Pos)
		cands = append(cands, targeting.Candidate{
			Entry:       en,
			Distance:    to.Len(),
			Bearing:     geom.AngleDiff(to.Angle(), a.Heading),
			PersistTick: st.persist[en.TargetID],
		})
	}
	return targeting.Select(a.ID, cands, a.Weights, a.MaxRange)
}
