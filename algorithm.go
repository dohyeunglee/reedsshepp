/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
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
*   * Neither the name of the Rice University nor the names of its
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
*********************************************************************/

package reedsshepp

import (
	"math"
)

const (
	twoPi  = 2 * math.Pi
	halfPi = 0.5 * math.Pi
)

var (
	epsilon         = math.Nextafter(1, 2) - 1 // C++ std::numeric_limits<double>::epsilon()
	zero            = 10 * epsilon
	pathCourseTypes = [18][5]PathCourseType{
		{CourseLeft, CourseRight, CourseLeft, CourseNone, CourseNone},      // 0
		{CourseRight, CourseLeft, CourseRight, CourseNone, CourseNone},     // 1
		{CourseLeft, CourseRight, CourseLeft, CourseRight, CourseNone},     // 2
		{CourseRight, CourseLeft, CourseRight, CourseLeft, CourseNone},     // 3
		{CourseLeft, CourseRight, CourseStraight, CourseLeft, CourseNone},  // 4
		{CourseRight, CourseLeft, CourseStraight, CourseRight, CourseNone}, // 5
		{CourseLeft, CourseStraight, CourseRight, CourseLeft, CourseNone},  // 6
		{CourseRight, CourseStraight, CourseLeft, CourseRight, CourseNone}, // 7
		{CourseLeft, CourseRight, CourseStraight, CourseRight, CourseNone}, // 8
		{CourseRight, CourseLeft, CourseStraight, CourseLeft, CourseNone},  // 9
		{CourseRight, CourseStraight, CourseRight, CourseLeft, CourseNone}, // 10
		{CourseLeft, CourseStraight, CourseLeft, CourseRight, CourseNone},  // 11
		{CourseLeft, CourseStraight, CourseRight, CourseNone, CourseNone},  // 12
		{CourseRight, CourseStraight, CourseLeft, CourseNone, CourseNone},  // 13
		{CourseLeft, CourseStraight, CourseLeft, CourseNone, CourseNone},   // 14
		{CourseRight, CourseStraight, CourseRight, CourseNone, CourseNone}, // 15
		{CourseLeft, CourseRight, CourseStraight, CourseLeft, CourseRight}, // 16
		{CourseRight, CourseLeft, CourseStraight, CourseRight, CourseLeft}, // 17
	}
)

type PathCourseType int

const (
	CourseNone PathCourseType = iota
	CourseLeft
	CourseStraight
	CourseRight
)

func reedsSheppPaths(x float64, y float64, phi float64) []Path {
	paths := make([]Path, 0)
	paths = append(paths, csc(x, y, phi)...)
	paths = append(paths, ccc(x, y, phi)...)
	paths = append(paths, cccc(x, y, phi)...)
	paths = append(paths, ccsc(x, y, phi)...)
	paths = append(paths, ccscc(x, y, phi)...)
	return paths
}

func lpSpLp(x float64, y float64, phi float64) (t float64, u float64, v float64, ok bool) {
	u, t = polar(x-math.Sin(phi), y-1+math.Cos(phi))
	if t >= -zero {
		v = mod2pi(phi - t)
		if v >= -zero {
			// TODO: assertion
			// assert(fabs(u * cos(t) + sin(phi) - x) < RS_EPS);
			// assert(fabs(u * sin(t) - cos(phi) + 1 - y) < RS_EPS);
			// assert(fabs(mod2pi(t + v - phi)) < RS_EPS);
			ok = true
		}
	}
	return
}

func lpSpRp(x float64, y float64, phi float64) (t float64, u float64, v float64, ok bool) {
	u1, t1 := polar(x+math.Sin(phi), y-1-math.Cos(phi))
	u1 = u1 * u1
	if u1 >= 4 {
		u = math.Sqrt(u1 - 4)
		theta := math.Atan2(2, u)
		t = mod2pi(t1 + theta)
		v = mod2pi(t - phi)
		// TODO: assertion
		// assert(fabs(2 * sin(t) + u * cos(t) - sin(phi) - x) < RS_EPS);
		// assert(fabs(-2 * cos(t) + u * sin(t) + cos(phi) + 1 - y) < RS_EPS);
		// assert(fabs(mod2pi(t - v - phi)) < RS_EPS);
		ok = t >= -zero && v >= -zero
	}
	return
}

func csc(x float64, y float64, phi float64) []Path {
	paths := make([]Path, 0, 8)

	if t, u, v, ok := lpSpLp(x, y, phi); ok {
		paths = append(paths, Path{t: t, u: u, v: v, courseTypesIndex: 14})
	}
	if t, u, v, ok := lpSpLp(-x, y, -phi); ok { // timeflip
		paths = append(paths, Path{t: -t, u: -u, v: -v, courseTypesIndex: 14})
	}
	if t, u, v, ok := lpSpLp(x, -y, -phi); ok { // reflect
		paths = append(paths, Path{t: t, u: u, v: v, courseTypesIndex: 15})
	}
	if t, u, v, ok := lpSpLp(-x, -y, phi); ok { // timeflip + reflect
		paths = append(paths, Path{t: -t, u: -u, v: -v, courseTypesIndex: 15})
	}

	if t, u, v, ok := lpSpRp(x, y, phi); ok {
		paths = append(paths, Path{t: t, u: u, v: v, courseTypesIndex: 12})
	}
	if t, u, v, ok := lpSpRp(-x, y, -phi); ok { // timeflip
		paths = append(paths, Path{t: -t, u: -u, v: -v, courseTypesIndex: 12})
	}
	if t, u, v, ok := lpSpRp(x, -y, -phi); ok { // reflect
		paths = append(paths, Path{t: t, u: u, v: v, courseTypesIndex: 13})
	}
	if t, u, v, ok := lpSpRp(-x, -y, phi); ok { // timeflip + reflect
		paths = append(paths, Path{t: -t, u: -u, v: -v, courseTypesIndex: 13})
	}

	return paths
}

func lpRmL(x float64, y float64, phi float64) (t float64, u float64, v float64, ok bool) {
	xi := x - math.Sin(phi)
	eta := y - 1 + math.Cos(phi)
	u1, theta := polar(xi, eta)
	if u1 <= 4 {
		u = -2 * math.Asin(0.25*u1)
		t = mod2pi(theta + 0.5*u + math.Pi)
		v = mod2pi(phi - t + u)
		// TODO: assertion
		// assert(fabs(2 * (sin(t) - sin(t - u)) + sin(phi) - x) < RS_EPS);
		// assert(fabs(2 * (-cos(t) + cos(t - u)) - cos(phi) + 1 - y) < RS_EPS);
		// assert(fabs(mod2pi(t - u + v - phi)) < RS_EPS);
		ok = t >= -zero && u <= zero
	}
	return
}

func ccc(x float64, y float64, phi float64) []Path {
	paths := make([]Path, 0, 8)

	if t, u, v, ok := lpRmL(x, y, phi); ok {
		paths = append(paths, Path{t: t, u: u, v: v, courseTypesIndex: 0})
	}
	if t, u, v, ok := lpRmL(-x, y, -phi); ok { // timeflip
		paths = append(paths, Path{t: -t, u: -u, v: -v, courseTypesIndex: 0})
	}
	if t, u, v, ok := lpRmL(x, -y, -phi); ok { // reflect
		paths = append(paths, Path{t: t, u: u, v: v, courseTypesIndex: 1})
	}
	if t, u, v, ok := lpRmL(-x, -y, phi); ok { // timeflip + reflect
		paths = append(paths, Path{t: -t, u: -u, v: -v, courseTypesIndex: 1})
	}

	// backwards
	xb := x*math.Cos(phi) + y*math.Sin(phi)
	yb := x*math.Sin(phi) - y*math.Cos(phi)
	if t, u, v, ok := lpRmL(xb, yb, phi); ok {
		paths = append(paths, Path{t: v, u: u, v: t, courseTypesIndex: 0})
	}
	if t, u, v, ok := lpRmL(-xb, yb, -phi); ok { // timeflip
		paths = append(paths, Path{t: -v, u: -u, v: -t, courseTypesIndex: 0})
	}
	if t, u, v, ok := lpRmL(xb, -yb, -phi); ok { // reflect
		paths = append(paths, Path{t: v, u: u, v: t, courseTypesIndex: 1})
	}
	if t, u, v, ok := lpRmL(-xb, -yb, phi); ok { // timeflip + reflect
		paths = append(paths, Path{t: -v, u: -u, v: -t, courseTypesIndex: 1})
	}

	return paths
}

func lpRupLumRm(x float64, y float64, phi float64) (t float64, u float64, v float64, ok bool) {
	xi := x + math.Sin(phi)
	eta := y - 1 - math.Cos(phi)
	rho := 0.25 * (2 + math.Sqrt(xi*xi+eta*eta))
	if rho <= 1 {
		u = math.Acos(rho)
		t, v = tauOmega(u, -u, xi, eta, phi)
		// TODO: assertion
		// assert(fabs(2 * (sin(t) - sin(t - u) + sin(t - 2 * u)) - sin(phi) - x) < RS_EPS);
		// assert(fabs(2 * (-cos(t) + cos(t - u) - cos(t - 2 * u)) + cos(phi) + 1 - y) < RS_EPS);
		// assert(fabs(mod2pi(t - 2 * u - v - phi)) < RS_EPS);
		ok = t >= -zero && v <= zero
	}
	return
}

func lpRumLumRp(x float64, y float64, phi float64) (t float64, u float64, v float64, ok bool) {
	xi := x + math.Sin(phi)
	eta := y - 1 - math.Cos(phi)
	rho := (20 - xi*xi - eta*eta) / 16
	if rho >= 0 && rho <= 1 {
		u = -math.Acos(rho)
		if u >= -halfPi {
			t, v = tauOmega(u, u, xi, eta, phi)
			ok = t >= -zero && v >= -zero
		}
	}
	return
}

func cccc(x float64, y float64, phi float64) []Path {
	paths := make([]Path, 0, 8)

	if t, u, v, ok := lpRupLumRm(x, y, phi); ok {
		paths = append(paths, Path{t: t, u: u, v: -u, w: v, courseTypesIndex: 2})
	}
	if t, u, v, ok := lpRupLumRm(-x, y, -phi); ok { // timeflip
		paths = append(paths, Path{t: -t, u: -u, v: u, w: -v, courseTypesIndex: 2})
	}
	if t, u, v, ok := lpRupLumRm(x, -y, -phi); ok { // reflect
		paths = append(paths, Path{t: t, u: u, v: -u, w: v, courseTypesIndex: 3})
	}
	if t, u, v, ok := lpRupLumRm(-x, -y, phi); ok { // timeflip + reflect
		paths = append(paths, Path{t: -t, u: -u, v: u, w: -v, courseTypesIndex: 3})
	}

	if t, u, v, ok := lpRumLumRp(x, y, phi); ok {
		paths = append(paths, Path{t: t, u: u, v: u, w: v, courseTypesIndex: 2})
	}
	if t, u, v, ok := lpRumLumRp(-x, y, -phi); ok { // timeflip
		paths = append(paths, Path{t: -t, u: -u, v: -u, w: -v, courseTypesIndex: 2})
	}
	if t, u, v, ok := lpRumLumRp(x, -y, -phi); ok { // reflect
		paths = append(paths, Path{t: t, u: u, v: u, w: v, courseTypesIndex: 3})
	}
	if t, u, v, ok := lpRumLumRp(-x, -y, phi); ok { // timeflip + reflect
		paths = append(paths, Path{t: -t, u: -u, v: -u, w: -v, courseTypesIndex: 3})
	}

	return paths
}

func lpRmSmLm(x float64, y float64, phi float64) (t float64, u float64, v float64, ok bool) {
	xi := x - math.Sin(phi)
	eta := y - 1 + math.Cos(phi)
	rho, theta := polar(xi, eta)
	if rho >= 2 {
		r := math.Sqrt(rho*rho - 4)
		u = 2 - r
		t = mod2pi(theta + math.Atan2(r, -2))
		v = mod2pi(phi - halfPi - t)
		// TODO: assertion
		// assert(fabs(2 * (sin(t) - cos(t)) - u * sin(t) + sin(phi) - x) < RS_EPS);
		// assert(fabs(-2 * (sin(t) + cos(t)) + u * cos(t) - cos(phi) + 1 - y) < RS_EPS);
		// assert(fabs(mod2pi(t + pi / 2 + v - phi)) < RS_EPS);
		ok = t >= -zero && u <= zero && v <= zero
	}
	return
}

func lpRmSmRm(x float64, y float64, phi float64) (t float64, u float64, v float64, ok bool) {
	xi := x + math.Sin(phi)
	eta := y - 1 - math.Cos(phi)
	rho, theta := polar(-eta, xi)
	if rho >= 2 {
		t = theta
		u = 2 - rho
		v = mod2pi(t + halfPi - phi)
		// TODO: assertion
		// assert(fabs(2*sin(t)-cos(t-v)-u*sin(t) - x) < RS_EPS);
		// assert(fabs(-2*cos(t)-sin(t-v)+u*cos(t)+1 - y) < RS_EPS);
		// assert(fabs(mod2pi(t+pi/2-v-phi)) < RS_EPS);
		ok = t >= -zero && u <= zero && v <= zero
	}
	return
}

func ccsc(x float64, y float64, phi float64) []Path {
	paths := make([]Path, 0, 16)

	if t, u, v, ok := lpRmSmLm(x, y, phi); ok {
		paths = append(paths, Path{t: t, u: -halfPi, v: u, w: v, courseTypesIndex: 4})
	}
	if t, u, v, ok := lpRmSmLm(-x, y, -phi); ok { // timeflip
		paths = append(paths, Path{t: -t, u: halfPi, v: -u, w: -v, courseTypesIndex: 4})
	}
	if t, u, v, ok := lpRmSmLm(x, -y, -phi); ok { // reflect
		paths = append(paths, Path{t: t, u: -halfPi, v: u, w: v, courseTypesIndex: 5})
	}
	if t, u, v, ok := lpRmSmLm(-x, -y, phi); ok { // timeflip + reflect
		paths = append(paths, Path{t: -t, u: halfPi, v: -u, w: -v, courseTypesIndex: 5})
	}

	if t, u, v, ok := lpRmSmRm(x, y, phi); ok {
		paths = append(paths, Path{t: t, u: -halfPi, v: u, w: v, courseTypesIndex: 8})
	}
	if t, u, v, ok := lpRmSmRm(-x, y, -phi); ok { // timeflip
		paths = append(paths, Path{t: -t, u: halfPi, v: -u, w: -v, courseTypesIndex: 8})
	}
	if t, u, v, ok := lpRmSmRm(x, -y, -phi); ok { // reflect
		paths = append(paths, Path{t: t, u: -halfPi, v: u, w: v, courseTypesIndex: 9})
	}
	if t, u, v, ok := lpRmSmRm(-x, -y, phi); ok { // timeflip + reflect
		paths = append(paths, Path{t: -t, u: halfPi, v: -u, w: -v, courseTypesIndex: 9})
	}

	// backwards
	xb := x*math.Cos(phi) + y*math.Sin(phi)
	yb := x*math.Sin(phi) - y*math.Cos(phi)
	if t, u, v, ok := lpRmSmLm(xb, yb, phi); ok {
		paths = append(paths, Path{t: v, u: u, v: -halfPi, w: t, courseTypesIndex: 6})
	}
	if t, u, v, ok := lpRmSmLm(-xb, yb, -phi); ok { // timeflip
		paths = append(paths, Path{t: -v, u: -u, v: halfPi, w: -t, courseTypesIndex: 6})
	}
	if t, u, v, ok := lpRmSmLm(xb, -yb, -phi); ok { // reflect
		paths = append(paths, Path{t: v, u: u, v: -halfPi, w: t, courseTypesIndex: 7})
	}
	if t, u, v, ok := lpRmSmLm(-xb, -yb, phi); ok { // timeflip + reflect
		paths = append(paths, Path{t: -v, u: -u, v: halfPi, w: -t, courseTypesIndex: 7})
	}

	if t, u, v, ok := lpRmSmRm(xb, yb, phi); ok {
		paths = append(paths, Path{t: v, u: u, v: -halfPi, w: t, courseTypesIndex: 10})
	}
	if t, u, v, ok := lpRmSmRm(-xb, yb, -phi); ok { // timeflip
		paths = append(paths, Path{t: -v, u: -u, v: halfPi, w: -t, courseTypesIndex: 10})
	}
	if t, u, v, ok := lpRmSmRm(xb, -yb, -phi); ok { // reflect
		paths = append(paths, Path{t: v, u: u, v: -halfPi, w: t, courseTypesIndex: 11})
	}
	if t, u, v, ok := lpRmSmRm(-xb, -yb, phi); ok { // timeflip + reflect
		paths = append(paths, Path{t: -v, u: -u, v: halfPi, w: -t, courseTypesIndex: 11})
	}

	return paths
}

func lpRmSLmRp(x float64, y float64, phi float64) (t float64, u float64, v float64, ok bool) {
	xi := x + math.Sin(phi)
	eta := y - 1 - math.Cos(phi)
	rho, _ := polar(xi, eta)
	if rho >= 2 {
		u = 4 - math.Sqrt(rho*rho-4)
		if u <= zero {
			t = mod2pi(math.Atan2((4-u)*xi-2*eta, -2*xi+(u-4)*eta))
			v = mod2pi(t - phi)
			// TODO: assertion
			// assert(fabs(4 * sin(t) - 2 * cos(t) - u * sin(t) - sin(phi) - x) < RS_EPS);
			// assert(fabs(-4 * cos(t) - 2 * sin(t) + u * cos(t) + cos(phi) + 1 - y) < RS_EPS);
			// assert(fabs(mod2pi(t - v - phi)) < RS_EPS);
			ok = t >= -zero && v >= -zero
		}
	}
	return
}

func ccscc(x float64, y float64, phi float64) []Path {
	paths := make([]Path, 0, 4)

	if t, u, v, ok := lpRmSLmRp(x, y, phi); ok {
		paths = append(paths, Path{t: t, u: -halfPi, v: u, w: -halfPi, x: v, courseTypesIndex: 16})
	}
	if t, u, v, ok := lpRmSLmRp(-x, y, -phi); ok { // timeflip
		paths = append(paths, Path{t: -t, u: halfPi, v: -u, w: halfPi, x: -v, courseTypesIndex: 16})
	}
	if t, u, v, ok := lpRmSLmRp(x, -y, -phi); ok { // reflect
		paths = append(paths, Path{t: t, u: -halfPi, v: u, w: -halfPi, x: v, courseTypesIndex: 17})
	}
	if t, u, v, ok := lpRmSLmRp(-x, -y, phi); ok { // timeflip + reflect
		paths = append(paths, Path{t: -t, u: halfPi, v: -u, w: halfPi, x: -v, courseTypesIndex: 17})
	}

	return paths
}

func mod2pi(x float64) float64 {
	v := math.Mod(x, twoPi)
	if v < -math.Pi {
		v += twoPi
	} else if v > math.Pi {
		v -= twoPi
	}
	return v
}

func polar(x float64, y float64) (r float64, theta float64) {
	r = math.Sqrt(x*x + y*y)
	theta = math.Atan2(y, x)
	return
}

func tauOmega(u float64, v float64, xi float64, eta float64, phi float64) (tau float64, omega float64) {
	delta := mod2pi(u - v)
	A := math.Sin(u) - math.Sin(delta)
	B := math.Cos(u) - math.Cos(delta) - 1
	t1 := math.Atan2(eta*A-xi*B, xi*A+eta*B)
	t2 := 2*(math.Cos(delta)-math.Cos(v)-math.Cos(u)) + 3

	if t2 < 0 {
		tau = mod2pi(t1 + math.Pi)
	} else {
		tau = mod2pi(t1)
	}

	omega = mod2pi(tau - u + v - phi)
	return
}
