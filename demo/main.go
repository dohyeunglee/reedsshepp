/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, Guan-Horng Liu.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the the copyright holder nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author:  Guan-Horng Liu
/********************************************************************/

package main

import (
	"fmt"
	"math"

	"gonum.org/v1/plot"
	"gonum.org/v1/plot/plotter"
	"gonum.org/v1/plot/vg"
	"gonum.org/v1/plot/vg/draw"
	"image/color"

	"github.com/dohyeunglee/reedsshepp"
)

const (
	stepSize      = 0.5
	turningRadius = 5.8
)

var (
	examples = []struct {
		Name    string
		Start   reedsshepp.State
		Goal    reedsshepp.State
		AxisMin [2]float64
		AxisMax [2]float64
	}{
		{
			Name:  "(0, 0, 0) -> (0, 0, pi)",
			Start: reedsshepp.State{X: 0, Y: 0, Yaw: 0}, Goal: reedsshepp.State{X: 0, Y: 0, Yaw: math.Pi},
			AxisMin: [2]float64{-1, -3},
			AxisMax: [2]float64{5, 3},
		},
		{
			Name:  "(0, 0, pi) -> (3, 4, 0)",
			Start: reedsshepp.State{X: 0, Y: 0, Yaw: math.Pi / 4}, Goal: reedsshepp.State{X: 3, Y: 4, Yaw: 0},
			AxisMin: [2]float64{0, -1},
			AxisMax: [2]float64{4, 5},
		},
		{
			Name:  "(4, 4, pi/4) -> (0, 4, 0)",
			Start: reedsshepp.State{X: 4, Y: 4, Yaw: math.Pi / 4}, Goal: reedsshepp.State{X: 0, Y: 4, Yaw: 0},
			AxisMin: [2]float64{-1, 1},
			AxisMax: [2]float64{5, 7},
		},
		{
			Name:  "(4, 0, 0) -> (-3, 4, pi)",
			Start: reedsshepp.State{X: 4, Y: 0, Yaw: 0}, Goal: reedsshepp.State{X: -3, Y: 4, Yaw: math.Pi},
			AxisMin: [2]float64{-4, -4},
			AxisMax: [2]float64{6, 6},
		},
		{
			Name:  "(-4, 0, 0) -> (3, 4, pi/3)",
			Start: reedsshepp.State{X: -4, Y: 0, Yaw: 0}, Goal: reedsshepp.State{X: 3, Y: 4, Yaw: math.Pi / 3},
			AxisMin: [2]float64{-5, -2},
			AxisMax: [2]float64{4, 6},
		},
		{
			Name:  "(4, 4, 0) -> (3, 4, pi/2)",
			Start: reedsshepp.State{X: 4, Y: 4, Yaw: 0}, Goal: reedsshepp.State{X: 3, Y: 4, Yaw: math.Pi / 2},
			AxisMin: [2]float64{2, 1},
			AxisMax: [2]float64{6, 5},
		},
	}
)

func getPoint(centerX float64, centerY float64, radius float64, orin float64) (x float64, y float64) {
	x = centerX + radius*math.Cos(orin)
	y = centerY + radius*math.Sin(orin)
	return
}

func plotCar(p *plot.Plot, state reedsshepp.State) {
	aX, aY := getPoint(state.X, state.Y, stepSize, state.Yaw)
	bX, bY := getPoint(state.X, state.Y, stepSize/2, state.Yaw+150.0/180.0*math.Pi)
	cX, cY := getPoint(state.X, state.Y, stepSize/2, state.Yaw-150.0/180.0*math.Pi)

	points := plotter.XYs{
		{X: aX, Y: aY},
		{X: bX, Y: bY},
		{X: cX, Y: cY},
		{X: aX, Y: aY},
	}

	line, err := plotter.NewLine(points)
	if err != nil {
		panic(err)
	}

	line.Color = color.RGBA{G: 255, A: 255}
	line.Width = vg.Points(1.5)

	p.Add(line)
}

func plotPath(p *plot.Plot, states []reedsshepp.StateWithDirection) {
	points := make(plotter.XYs, 0, len(states))
	for _, state := range states {
		points = append(points, plotter.XY{X: state.X, Y: state.Y})
	}

	lpLine, lpPoints, err := plotter.NewLinePoints(points)
	if err != nil {
		panic(err)
	}

	lpLine.Color = color.RGBA{B: 255, A: 255}
	lpPoints.Color = color.RGBA{R: 255, A: 255}
	lpPoints.Shape = draw.CircleGlyph{}

	p.Add(lpLine, lpPoints)
}

func main() {
	for i, example := range examples {
		path, _ := reedsshepp.MinLengthPath(example.Start, example.Goal, turningRadius)
		interpolated := path.Interpolate(stepSize)

		p := plot.New()
		p.Title.Text = fmt.Sprintf("ReedsSheppPath Demo: %s\nLength: %.2f", example.Name, path.Length())
		p.X.Label.Text = "X"
		p.Y.Label.Text = "Y"
		p.Add(plotter.NewGrid())
		p.X.Min = example.AxisMin[0]
		p.Y.Min = example.AxisMin[1]
		p.X.Max = example.AxisMax[0]
		p.Y.Max = example.AxisMax[1]

		plotCar(p, example.Start)
		plotCar(p, example.Goal)

		plotPath(p, interpolated)

		if err := p.Save(4*vg.Inch, 4*vg.Inch, fmt.Sprintf("demo_path%d.png", i)); err != nil {
			panic(err)
		}
	}
}
