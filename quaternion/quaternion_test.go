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

func TestScale(t *testing.T) {
	q00.Scale(s00)
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
	q20.Normalize()
	if q20 != q21 {
		t.Fail()
	}
}

func TestRandom(t *testing.T) {
	_ = Random()
	t.Skip() // skip the test for random quanternion generator (no viable testing available)
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
