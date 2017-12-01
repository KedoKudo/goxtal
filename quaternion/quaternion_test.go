/*
Testing for the quanternion packages
*/

package quaternion

import (
	"fmt"
	"math"
	"testing"
)

var (
	q00 = Quaternion{W: 1, X: 0, Y: 0, Z: 0}
	q01 = Quaternion{W: 5, X: 0, Y: 0, Z: 0}
	s00 = 5.0

	q10 = Quaternion{W: 2, X: 0, Y: 0, Z: 0}
	s10 = 2.0

	q20 = Quaternion{W: 2, X: 0, Y: 0, Z: 0}
	q21 = Quaternion{W: 1, X: 0, Y: 0, Z: 0}

	vecBeforeRot = [3]float64{1, 1, 0}
	eulersForRot = [3]float64{45, 0, 0}
	q50          = FromBungeEulers(eulersForRot, true)
	vRotedTarget = [3]float64{0, math.Sqrt(2), 0}
)

func TestScaledBy(t *testing.T) {
	q00.ScaledBy(s00)
	if q00 != q01 {
		t.Fail()
	}
}

func TestNorm(t *testing.T) {
	if q10.Norm() != s10 {
		t.Fail()
	}
}

func TestNormalize(t *testing.T) {
	qtmp := q20.Normalize()
	if qtmp != q21 {
		t.Fail()
	}
}

func TestNormalized(t *testing.T) {
	q20.Normalized()
	if q20 != q21 {
		t.Fail()
	}
}

func TestConjugate(t *testing.T) {
	q := Random()
	qstar := q.Conjugate()

	err := (q.W - qstar.W) + (q.X + qstar.X) + (q.Y + qstar.Y) + (q.Z + qstar.Z)

	if math.Abs(err) > 1e-10 {
		t.Fail()
	}
}

func TestConjugated(t *testing.T) {
	q := Random()
	qstar := q.Conjugate()
	q.Conjugated()

	if q != qstar {
		t.Fail()
	}
}

func TestRandom(t *testing.T) {
	_ = Random()
	t.Skip() // skip the test for random quanternion generator (no viable testing available)
}

func TestAsArray(t *testing.T) {
	q := Random()
	_ = q.AsArray()
	t.Skip()
}

func TestFromEulers(t *testing.T) {
	ang := math.Pi / 4
	m := [3][3]float64{
		{math.Cos(ang), -math.Sin(ang), 0},
		{math.Sin(ang), math.Cos(ang), 0},
		{0, 0, 1},
	}

	q := FromBungeEulers([3]float64{ang, 0, 0}, false)
	mq := q.AsMatrix()

	err := 0.0
	for i, r := range mq {
		for j, v := range r {
			err = err + math.Abs(m[i][j]-v)
		}
	}

	if err > 1e-10 {
		t.Fail()
	}
}

func TestRotateVec(t *testing.T) {
	vRoted := q50.RotateVec(vecBeforeRot)
	err := 0.0

	for i, v := range vRoted {
		err = err + math.Abs(vRotedTarget[i]-v)
	}

	if err > 1e-10 {
		fmt.Println(vRotedTarget)
		fmt.Println(vRoted)
		fmt.Println(err)
		t.Fail()
	}
}

func TestMul(t *testing.T) {
	ang := math.Pi / 6
	q0 := FromBungeEulers([3]float64{ang, 0, 0}, false)
	q1 := FromBungeEulers([3]float64{0, ang, 0}, false)
	q3 := FromBungeEulers([3]float64{ang, ang, 0}, false)

	q := q0.Mul(q1)

	if q.Diff(q3) > 1e-10 {
		t.Fail()
	}
}

func TestFromAngleAxis(t *testing.T) {
	ang := math.Pi / 3
	axis := [3]float64{1 / math.Sqrt(2), 1 / math.Sqrt(2), 0}
	qtarget := Quaternion{
		W: math.Cos(ang / 2),
		X: math.Sin(ang/2) * axis[0],
		Y: math.Sin(ang/2) * axis[1],
		Z: math.Sin(ang/2) * axis[2],
	}

	q := FromAngleAxis(ang, [3]float64{1, 1, 0}, false)

	if q.Diff(qtarget) > 1e-10 {
		fmt.Println(q)
		fmt.Println(axis)
		fmt.Println(qtarget)
		t.Fail()
	}

	angDegree := ang / math.Pi * 180
	q2 := FromAngleAxis(angDegree, [3]float64{1, 1, 0}, true)

	if q2.Diff(qtarget) > 1e-10 {
		t.Fail()
	}
}
