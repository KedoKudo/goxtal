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
)

func TestScale(t *testing.T) {
	q00.Scale(s00)
	if q00 != q01 {
		t.Fail()
	}
}
