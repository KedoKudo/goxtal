/*
Testing for the quanternion packages
*/

package quaternion

import (
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
