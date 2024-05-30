package reedsshepp_test

import (
	"math"
	"math/rand"
	"testing"

	"github.com/stretchr/testify/assert"

	"github.com/dohyeunglee/reedsshepp"
)

// TestMinLengthPath compares the result of MinLengthPath and `distance` function of ompl.
func TestMinLengthPath(t *testing.T) {
	turningRadius := 5.8
	type args struct {
		start reedsshepp.State
		goal  reedsshepp.State
	}
	tests := []struct {
		name       string
		args       args
		want       float64
		wantNoPath bool
	}{
		{
			name: "if start and goal state are the same",
			args: args{
				start: reedsshepp.State{
					X:   0,
					Y:   0,
					Yaw: 0,
				},
				goal: reedsshepp.State{
					X:   0,
					Y:   0,
					Yaw: 0,
				},
			},
			wantNoPath: true,
		},
		{
			name: "(0, 0, 0) -> (0, 0, pi)",
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
			name: "(0, 0, pi/4) -> (3, 4, 0)",
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
			name: "(4, 4, pi/4) -> (0, 4, 0)",
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
			name: "(4, 0, 0) -> (-3, 4, pi)",
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
			name: "(-4, 0, 0) -> (3, 4, pi/3)",
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
			name: "(4, 4, 0) -> (3, 4, pi/2)",
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
		t.Run(tt.name, func(t *testing.T) {
			// WHEN
			path, ok := reedsshepp.MinLengthPath(tt.args.start, tt.args.goal, turningRadius)

			// THEN
			assert.Equal(t, !tt.wantNoPath, ok)
			if !tt.wantNoPath {
				assert.InDelta(t, tt.want, path.Length(), 1e-9)
			}
		})
	}
}

func BenchmarkMinLengthPath(b *testing.B) {
	turningRadius := 5.8

	for i := 0; i < b.N; i++ {
		b.StopTimer()
		start := reedsshepp.State{
			X:   rand.Float64() * 100,
			Y:   rand.Float64() * 100,
			Yaw: rand.Float64() * 2 * math.Pi,
		}
		goal := reedsshepp.State{
			X:   rand.Float64() * 100,
			Y:   rand.Float64() * 100,
			Yaw: rand.Float64() * 2 * math.Pi,
		}
		b.StartTimer()
		path, _ := reedsshepp.MinLengthPath(start, goal, turningRadius)
		_ = path.Length()
	}
}
