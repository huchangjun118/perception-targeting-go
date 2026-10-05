package geom

import "testing"

func TestRaySegmentHit(t *testing.T) {
	o := Vec2{0, 0}
	d := Vec2{1, 0}
	seg := Segment{A: Vec2{5, -1}, B: Vec2{5, 1}}
	tt, ok := RaySegmentHit(o, d, 10, seg)
	if !ok || tt != 5 {
		t.Fatalf("expected hit at t=5, got %v %v", tt, ok)
	}
	// 超出最大距离
	if _, ok := RaySegmentHit(o, d, 4, seg); ok {
		t.Fatal("expected miss beyond maxDist")
	}
	// 平行线段
	par := Segment{A: Vec2{0, 1}, B: Vec2{5, 1}}
	if _, ok := RaySegmentHit(o, d, 10, par); ok {
		t.Fatal("expected miss for parallel segment")
	}
	// 端点之外
	off := Segment{A: Vec2{5, 2}, B: Vec2{5, 3}}
	if _, ok := RaySegmentHit(o, d, 10, off); ok {
		t.Fatal("expected miss beyond segment end")
	}
}

func TestRayCircleHit(t *testing.T) {
	o := Vec2{0, 0}
	d := Vec2{1, 0}
	c := Circle{Center: Vec2{5, 0}, Radius: 1}
	tt, ok := RayCircleHit(o, d, 10, c)
	if !ok || tt != 4 {
		t.Fatalf("expected hit at t=4, got %v %v", tt, ok)
	}
	// 起点在圆内不命中
	if _, ok := RayCircleHit(Vec2{5, 0}, d, 10, c); ok {
		t.Fatal("expected miss when origin inside circle")
	}
	// 擦边（距离恰好等于半径）：判别式为 0，视为命中边界确定
	tan := Circle{Center: Vec2{5, 1}, Radius: 1}
	if _, ok := RayCircleHit(o, d, 10, tan); !ok {
		t.Fatal("expected deterministic hit for tangent ray")
	}
}

func TestOccluded(t *testing.T) {
	o := Vec2{0, 0}
	target := Vec2{10, 0}
	wall := []Segment{{A: Vec2{5, -2}, B: Vec2{5, 2}}}
	if !Occluded(o, target, wall, nil) {
		t.Fatal("expected occlusion by wall")
	}
	if Occluded(o, target, nil, nil) {
		t.Fatal("expected no occlusion without obstacles")
	}
	blocker := []Circle{{Center: Vec2{5, 0}, Radius: 1}}
	if !Occluded(o, target, nil, blocker) {
		t.Fatal("expected occlusion by dynamic circle")
	}
	// 遮挡体在目标背后不遮挡
	behind := []Circle{{Center: Vec2{15, 0}, Radius: 1}}
	if Occluded(o, target, nil, behind) {
		t.Fatal("expected no occlusion from circle behind target")
	}
}
