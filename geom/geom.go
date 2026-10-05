// Package geom 提供二维几何基础类型与确定性判定工具。
package geom

import "math"

// Vec2 二维向量。
type Vec2 struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
}

func (a Vec2) Add(b Vec2) Vec2 { return Vec2{a.X + b.X, a.Y + b.Y} }
func (a Vec2) Sub(b Vec2) Vec2 { return Vec2{a.X - b.X, a.Y - b.Y} }
func (a Vec2) Scale(s float64) Vec2 {
	return Vec2{a.X * s, a.Y * s}
}
func (a Vec2) Len() float64 { return math.Hypot(a.X, a.Y) }
func (a Vec2) Dist(b Vec2) float64 {
	return a.Sub(b).Len()
}

// Normalize 返回单位向量；零向量返回 (1,0) 以保证确定性。
func (a Vec2) Normalize() Vec2 {
	l := a.Len()
	if l == 0 {
		return Vec2{1, 0}
	}
	return Vec2{a.X / l, a.Y / l}
}

// Angle 返回向量方向角（弧度，[-pi, pi]）。
func (a Vec2) Angle() float64 { return math.Atan2(a.Y, a.X) }

// AngleDiff 返回 a-b 的最小夹角，范围 [-pi, pi]。
func AngleDiff(a, b float64) float64 {
	d := math.Mod(a-b+math.Pi, 2*math.Pi)
	if d < 0 {
		d += 2 * math.Pi
	}
	return d - math.Pi
}

// mathSqrt 内部包装，便于单点替换。
func mathSqrt(x float64) float64 { return math.Sqrt(x) }
