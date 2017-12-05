/*
Testing for the quanternion packages
*/

package orientation

import (
	"fmt"
	"math"
	"testing"
)

var (
	q = Quaternion{W: 1, X: 0, Y: 0, Z: 0}
)

func TestBasicDummy(t *testing.T) {
	// skip the trival function testing
	q := Quaternion{}
	q.Random()
	_ = q.Normalize()
	q.Conjugated()
	_ = q.Conjugate()
	_ = q.AsArray()
	_ = q.AsQuaternion()
}

func TestEulerConversion(t *testing.T) {
	// test general case
	eulers := [3]float64{45, 30, 10}

	q := Quaternion{}
	q.FromBungeEulers(eulers, true)

	eulersCalc := q.AsBungeEulers(true)

	diff := 0.0
	for i, e := range eulers {
		diff += math.Abs(e - eulersCalc[i])
	}

	if diff > 1e-8 {
		t.Fail()
	}

	// test corner case 1
	eulers = [3]float64{33, 0, 0}
	q.FromBungeEulers(eulers, true)
	eulersCalc = q.AsBungeEulers(true)
	diff = 0.0
	for i, e := range eulers {
		diff += math.Abs(e - eulersCalc[i])
	}

	if diff > 1e-8 {
		t.Fail()
	}

	// test corner case 2
	eulers = [3]float64{3, 180, 0}
	q.FromBungeEulers(eulers, true)
	eulersCalc = q.AsBungeEulers(true)
	diff = 0.0
	for i, e := range eulers {
		diff += math.Abs(e - eulersCalc[i])
	}

	if diff > 1e-8 {
		t.Fail()
	}
}

func TestAngleAxisConversion(t *testing.T) {
	q := Quaternion{}
	q.Random()

	ang, axis := q.AsAngleAxis(true)

	q1 := Quaternion{}
	q1.FromAngleAxis(ang, axis, true)

	if q1.Diff(q) > 1e-8 {
		t.Fail()
	}
}

func TestRotationMatrixConversion(t *testing.T) {
	q := Quaternion{}
	q.Random()

	m := q.AsMatrix()

	q1 := Quaternion{}
	q1.FromMatrix(m)

	if q1.Diff(q) > 1e-8 {
		fmt.Println(m)
		fmt.Println(q)
		fmt.Println(q1)
		t.Fail()
	}
}

func TestQuaternionMul(t *testing.T) {
	q0 := Quaternion{}
	q0.FromBungeEulers([3]float64{30, 0, 0}, true)

	q1 := Quaternion{}
	q1.FromBungeEulers([3]float64{0, 10, 0}, true)

	q2 := Quaternion{}
	q2.FromBungeEulers([3]float64{30, 10, 0}, true)

	if q2.Diff(q0.Mul(q1)) > 1e-8 {
		t.Fail()
	}
}

func TestRotateVec(t *testing.T) {
	q := Quaternion{}
	q.FromBungeEulers([3]float64{45, 0, 0}, true)

	vec := q.RotateVec([3]float64{1, 0, 0})

	vecRotated := [3]float64{math.Sqrt(2) / 2, math.Sqrt(2) / 2, 0}

	err := 0.0
	for i, v := range vec {
		err += math.Abs(v - vecRotated[i])
	}

	if err > 1e-8 {
		t.Fail()
	}

}

func TestNorm(t *testing.T) {
	q := Quaternion{W: 5, X: 0, Y: 0, Z: 0}

	if math.Abs(q.Norm()-5) > 1e-8 {
		t.Fail()
	}
}

func TestNormalized(t *testing.T) {
	q := Quaternion{W: 5, X: 0, Y: 0, Z: 0}
	q.Normalized()
	unitq := Quaternion{W: 1, X: 0, Y: 0, Z: 0}

	if unitq.Diff(q) > 1e-8 {
		t.Fail()
	}
}

func TestScaledBy(t *testing.T) {
	q.ScaledBy(5)
	qscaled := Quaternion{W: 5, X: 0, Y: 0, Z: 0}

	if q.Diff(qscaled) > 1e-8 {
		t.Fail()
	}
}
