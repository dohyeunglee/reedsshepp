package main

import (
	"fmt"
	"math"

	"github.com/dohyeunglee/reedsshepp"
)

func main() {
	start := reedsshepp.State{
		X:   0,
		Y:   0,
		Yaw: 0,
	}

	goal := reedsshepp.State{
		X:   1,
		Y:   1,
		Yaw: math.Pi,
	}

	turningRadius := 1.0

	path, ok := reedsshepp.MinLengthPath(start, goal, turningRadius)
	if !ok {
		fmt.Println("There is no available ReedsSheepPath.")
		return
	}

	fmt.Printf("[MinLength ReedsSheppPath from %s to %s]\n", start, goal)
	fmt.Println("- Length: ", path.Length())
	fmt.Println("- Segments: ", path.Segments())
	fmt.Println("- Interpolated: ", path.Interpolate(0.1))
}
