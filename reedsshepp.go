// Package reedsshepp implements ReedsSheppPath planning based on [ompl](https://github.com/ompl/ompl) library.
package reedsshepp

import (
	"math"
)

// Direction represents path's direction: forward or backward.
type Direction bool

const (
	ForwardDirection  Direction = true
	BackwardDirection Direction = false
)

// PathCourseType represents ReedsSheppPath segment course type: L, S, R.
type PathCourseType int

const (
	CourseTypeNone PathCourseType = iota
	CourseTypeLeft
	CourseTypeStraight
	CourseTypeRight
)

// State represents the 3D state, x, y coordinate and yaw angle.
type State struct {
	X   float64
	Y   float64
	Yaw float64
}

// StateWithDirection represents `State` information with `Direction` information
type StateWithDirection struct {
	State
	Direction Direction
}

type PathSegment struct {
	Length     float64
	CourseType PathCourseType
}

// Direction returns `Direction` information(forward or backward) about the segment.
func (s PathSegment) Direction() Direction {
	if s.Length >= 0 {
		return ForwardDirection
	}
	return BackwardDirection
}

// Path corresponds to ReedsSheppPath.
type Path struct {
	t                float64
	u                float64
	v                float64
	w                float64
	x                float64
	courseTypesIndex int

	turningRadius float64
	segments      []PathSegment
	totalLength   float64

	origin State
}

// Length returns the total length of ReedsSheppPath.
func (p *Path) Length() float64 {
	return p.totalLength
}

// Segments returns all `PathSegment` information about ReedsSheppPath.
func (p *Path) Segments() []PathSegment {
	return p.segments
}

// Interpolate interpolates ReedsSheppPath by `stepSize`, returning `StateWithDirection` list.
func (p *Path) Interpolate(stepSize float64) []StateWithDirection {
	states := make([]StateWithDirection, 0)

	for distance := 0.0; distance <= p.Length(); distance += stepSize {
		states = append(states, p.stateAtDistance(p.origin, distance))
	}

	return states
}

// IsZero checks whether `Path` is default(zero) value.
func (p *Path) IsZero() bool {
	return len(p.segments) == 0
}

// stateAtDistance returns the state which is `distance` far from `start` state.
func (p *Path) stateAtDistance(start State, distance float64) StateWithDirection {
	if distance <= 0 {
		return StateWithDirection{
			State:     start,
			Direction: p.segments[0].Direction(),
		}
	}

	distance = min(distance, p.Length())

	var direction Direction

	for _, segment := range p.segments {
		direction = segment.Direction()

		var deltaDistance float64
		if direction == ForwardDirection {
			deltaDistance = min(distance, segment.Length)
			distance -= deltaDistance
		} else {
			deltaDistance = max(-distance, segment.Length)
			distance += deltaDistance
		}

		start = stateAtDistance(start, deltaDistance, segment.CourseType, p.turningRadius)

		if distance <= 0 {
			break
		}
	}

	return StateWithDirection{
		State:     start,
		Direction: direction,
	}
}

func (p *Path) setTurningRadius(turningRadius float64) {
	p.turningRadius = turningRadius

	p.t *= turningRadius
	p.u *= turningRadius
	p.v *= turningRadius
	p.w *= turningRadius
	p.x *= turningRadius

	lengths := [5]float64{p.t, p.u, p.v, p.w, p.x}
	courseTypes := pathCourseTypes[p.courseTypesIndex]

	p.segments = make([]PathSegment, 0, len(courseTypes))
	for i := 0; i < len(courseTypes); i++ {
		if courseTypes[i] == CourseTypeNone {
			break
		}

		length := lengths[i]
		p.totalLength += math.Abs(length)
		p.segments = append(p.segments, PathSegment{
			Length:     length,
			CourseType: courseTypes[i],
		})
	}
}

func stateAtDistance(start State, deltaDistance float64, courseType PathCourseType, turningRadius float64) State {
	if deltaDistance == 0 {
		return start
	}

	deltaX := 0.0
	deltaY := 0.0
	deltaYaw := 0.0

	phi := deltaDistance / turningRadius

	switch courseType {
	case CourseTypeLeft:
		deltaX = turningRadius*math.Sin(start.Yaw+phi) - turningRadius*math.Sin(start.Yaw)
		deltaY = turningRadius*-math.Cos(start.Yaw+phi) + turningRadius*math.Cos(start.Yaw)
		deltaYaw = phi
	case CourseTypeRight:
		deltaX = turningRadius*-math.Sin(start.Yaw-phi) + turningRadius*math.Sin(start.Yaw)
		deltaY = turningRadius*math.Cos(start.Yaw-phi) - turningRadius*math.Cos(start.Yaw)
		deltaYaw = -phi
	case CourseTypeStraight:
		deltaX = deltaDistance * math.Cos(start.Yaw)
		deltaY = deltaDistance * math.Sin(start.Yaw)
	default:
		panic("unknown course type")
	}

	return State{
		X:   start.X + deltaX,
		Y:   start.Y + deltaY,
		Yaw: start.Yaw + deltaYaw,
	}
}

// MinLengthPath returns the shortest `ReedsSheppPath` among all possible paths from the `start` state to the `goal` state, given a `turningRadius`.
// In case there are multiple paths with the same length, one of them is randomly selected and returned.
// The shortest ReedsSheppPath length can be obtained using the `Length` function of the returned `ReedsSheppPath`.
func MinLengthPath(start State, goal State, turningRadius float64) (Path, bool) {
	paths := AvailablePaths(start, goal, turningRadius)
	if len(paths) == 0 {
		return Path{}, false
	}

	minLength := math.MaxFloat64
	var bestPath Path

	for _, path := range paths {
		length := path.Length()
		if length < minLength {
			bestPath = path
			minLength = length
		}
	}

	return bestPath, true
}

// AvailablePaths returns all possible ReedsSheppPaths that can reach the `goal` state from the `start` state, given a `turningRadius`.
func AvailablePaths(start State, goal State, turningRadius float64) []Path {
	dx := goal.X - start.X
	dy := goal.Y - start.Y
	dth := goal.Yaw - start.Yaw
	c := math.Cos(start.Yaw)
	s := math.Sin(start.Yaw)
	x := c*dx + s*dy
	y := -s*dx + c*dy

	paths := reedsSheppPaths(x/turningRadius, y/turningRadius, dth)
	for i := range paths {
		paths[i].setTurningRadius(turningRadius)
		paths[i].origin = start
	}
	return paths
}
