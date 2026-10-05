package geom

// eps 是几何判定的统一容差，保证边界判定确定性：
// 距离小于 eps 的交点视为不相交（避免起点处自相交）。
const eps = 1e-9

// Circle 圆形遮挡体/目标体。
type Circle struct {
	Center Vec2    `json:"center"`
	Radius float64 `json:"radius"`
}

// Segment 线段（用于静态障碍边）。
type Segment struct {
	A Vec2 `json:"a"`
	B Vec2 `json:"b"`
}

// RaySegmentHit 判定射线（起点 o，方向 d 单位向量，最大距离 maxDist）
// 是否与线段相交。相交时返回交点距离 t（eps < t < maxDist-eps 之外不算）。
// 边界规则确定：t <= eps 或 t >= maxDist 视为未命中。
func RaySegmentHit(o, d Vec2, maxDist float64, s Segment) (float64, bool) {
	e := s.B.Sub(s.A)
	den := d.X*e.Y - d.Y*e.X
	if den > -eps && den < eps {
		return 0, false // 平行或共线：统一视为未命中（边界确定）
	}
	f := o.Sub(s.A)
	// 解 o + t*d = A + u*e，即 t*d - u*e = -f。
	t := -(f.X*e.Y - f.Y*e.X) / den
	u := (f.X*d.Y - f.Y*d.X) / -den
	if t <= eps || t >= maxDist {
		return 0, false
	}
	if u < 0 || u > 1 {
		return 0, false
	}
	return t, true
}

// RayCircleHit 判定射线是否与圆相交，返回最近交点距离。
// 起点在圆内视为未命中（目标自身不遮挡自己）。
func RayCircleHit(o, d Vec2, maxDist float64, c Circle) (float64, bool) {
	oc := o.Sub(c.Center)
	b := oc.X*d.X + oc.Y*d.Y
	cc := oc.X*oc.X + oc.Y*oc.Y - c.Radius*c.Radius
	disc := b*b - cc
	if disc < 0 {
		return 0, false
	}
	sq := 0.0
	if disc > 0 {
		sq = mathSqrt(disc)
	}
	t := -b - sq
	if t <= eps || t >= maxDist {
		return 0, false
	}
	return t, true
}

// Occluded 判定从 o 到 target 的视线是否被任一遮挡体阻挡。
// segs 为静态障碍边，circles 为动态物体。目标自身圆不计入（调用方排除）。
func Occluded(o, target Vec2, segs []Segment, circles []Circle) bool {
	d := target.Sub(o)
	dist := d.Len()
	if dist <= eps {
		return false
	}
	dir := d.Scale(1 / dist)
	for _, s := range segs {
		if _, ok := RaySegmentHit(o, dir, dist, s); ok {
			return true
		}
	}
	for _, c := range circles {
		if _, ok := RayCircleHit(o, dir, dist, c); ok {
			return true
		}
	}
	return false
}
