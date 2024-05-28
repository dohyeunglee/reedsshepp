package reedsshepp_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/stretchr/testify/assert"

	"github.com/dohyeunglee/reedsshepp"
)

var epsilon = math.Nextafter(1, 2) - 1

func TestAvailablePaths(t *testing.T) {
	// GIVEN
	start := reedsshepp.State{
		X:   0,
		Y:   0,
		Yaw: 0,
	}
	goal := reedsshepp.State{
		X:   0,
		Y:   0,
		Yaw: math.Pi,
	}
	turningRadius := 5.8

	// WHEN
	paths := reedsshepp.AvailablePaths(start, goal, turningRadius)

	// THEN
	assert.Equal(t, 10, len(paths))
}

func TestMinLengthPath(t *testing.T) {
	turningRadius := 5.8
	type args struct {
		start reedsshepp.State
		goal  reedsshepp.State
	}
	tests := []struct {
		args args
		want float64
	}{
		{
			args: args{
				start: reedsshepp.State{
					X:   0,
					Y:   0,
					Yaw: 0,
				},
				goal: reedsshepp.State{
					X:   0,
					Y:   0,
					Yaw: math.Pi,
				},
			},
			want: 18.221237390820797,
		},
		{
			args: args{
				start: reedsshepp.State{
					X:   0,
					Y:   0,
					Yaw: math.Pi / 4,
				},
				goal: reedsshepp.State{
					X:   3,
					Y:   4,
					Yaw: 0,
				},
			},
			want: 7.967765057618429,
		},
		{
			args: args{
				start: reedsshepp.State{
					X:   4,
					Y:   4,
					Yaw: math.Pi / 4,
				},
				goal: reedsshepp.State{
					X:   0,
					Y:   4,
					Yaw: 0,
				},
			},
			want: 6.389556686872893,
		},
		{
			args: args{
				start: reedsshepp.State{
					X:   4,
					Y:   0,
					Yaw: 0,
				},
				goal: reedsshepp.State{
					X:   -3,
					Y:   4,
					Yaw: math.Pi,
				},
			},
			want: 18.2212373908208,
		},
		{
			args: args{
				start: reedsshepp.State{
					X:   -4,
					Y:   0,
					Yaw: 0,
				},
				goal: reedsshepp.State{
					X:   3,
					Y:   4,
					Yaw: math.Pi / 3,
				},
			},
			want: 8.336208440323265,
		},
		{
			args: args{
				start: reedsshepp.State{
					X:   4,
					Y:   4,
					Yaw: 0,
				},
				goal: reedsshepp.State{
					X:   3,
					Y:   4,
					Yaw: math.Pi / 2,
				},
			},
			want: 9.1106186954104,
		},
	}

	for _, tt := range tests {
		// WHEN
		path, ok := reedsshepp.MinLengthPath(tt.args.start, tt.args.goal, turningRadius)

		// THEN
		assert.True(t, ok)
		assert.InEpsilon(t, tt.want, path.Length(), 10*epsilon)
	}
}

func TestPath_Interpolate(t *testing.T) {
	// GIVEN
	start := reedsshepp.State{
		X:   0,
		Y:   0,
		Yaw: 0,
	}
	goal := reedsshepp.State{
		X:   0,
		Y:   0,
		Yaw: math.Pi,
	}
	turningRadius := 5.8
	stepSize := 0.1

	// WHEN
	paths := reedsshepp.AvailablePaths(start, goal, turningRadius)
	path := paths[0]
	states := path.Interpolate(stepSize)

	// THEN
	fmt.Println(states)
	fmt.Println(len(states))
}