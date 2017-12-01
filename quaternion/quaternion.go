/*
Quanternion calculation for manipulating cyxtal orientations
rotations.
*/

package quaternion

import (
	"math"
	"math/rand"
	"time"
)

// Quaternion represents  q = w + Xi + Yj + Zk
type Quaternion struct {
	W float64 // cos(theta/2)
	X float64 // vx
	Y float64 // vy
	Z float64 // vz
}

// Norm returns the norm of the quaternion
func (q *Quaternion) Norm() float64 {
	return math.Sqrt(q.W*q.W + q.X*q.X + q.Y*q.Y + q.Z*q.Z)
}

// Normalize  normalize quanternion norm to 1 (necessary for rotation)
func (q *Quaternion) Normalize() {
	var qnorm = q.Norm()

	q.W = q.W / qnorm
	q.X = q.X / qnorm
	q.Y = q.Y / qnorm
	q.Z = q.Z / qnorm

}

// Mul multiply self with new quaternion, representing continuous rotation
func (q *Quaternion) Mul(q2 Quaternion) Quaternion {
	Aw := q.W
	Ax := q.X
	Ay := q.Y
	Az := q.Z
	Bw := q2.W
	Bx := q2.X
	By := q2.Y
	Bz := q2.Z

	return Quaternion{
		W: -Ax*Bx - Ay*By - Az*Bz + Aw*Bw,
		X: +Ax*Bw + Ay*Bz - Az*By + Aw*Bx,
		Y: -Ax*Bz + Ay*Bw + Az*Bx + Aw*By,
		Z: +Ax*By - Ay*Bx + Az*Bw + Aw*Bz,
	}
}

// AsMatrix returns the rotation matrix equivalent of given quaternion
func (q *Quaternion) AsMatrix() [3][3]float64 {
	m := [3][3]float64{}

	m[0][0] = 1 - 2*(q.Y*q.Y+q.Z*q.Z)
	m[0][1] = 2 * (q.X*q.Y - q.W*q.Z)
	m[0][2] = 2 * (q.W*q.Y + q.X*q.Z)

	m[1][0] = 2 * (q.W*q.Z + q.Y*q.X)
	m[1][1] = 1 - 2*(q.Z*q.Z+q.X*q.X)
	m[1][2] = 2 * (q.Y*q.Z - q.W*q.X)

	m[2][0] = 2 * (q.Z*q.X - q.W*q.Y)
	m[2][1] = 2 * (q.W*q.X + q.Z*q.Y)
	m[2][2] = 1 - 2*(q.X*q.X+q.Y*q.Y)

	return m
}

// RotateVec rotates a vector using the quaternion
func (q *Quaternion) RotateVec(v [3]float64) [3]float64 {
	var rotatedV [3]float64

	w := q.W
	x := q.X
	y := q.Y
	z := q.Z
	vx := v[0]
	vy := v[1]
	vz := v[2]

	rotatedV[0] = w*w*vx + 2*y*w*vz - 2*z*w*vy + x*x*vx + 2*y*x*vy + 2*z*x*vz - z*z*vx - y*y*vx
	rotatedV[1] = 2*x*y*vx + y*y*vy + 2*z*y*vz + 2*w*z*vx - z*z*vy + w*w*vy - 2*x*w*vz - x*x*vy
	rotatedV[2] = 2*x*z*vx + 2*y*z*vy + z*z*vz - 2*w*y*vx - y*y*vz + 2*w*x*vy - x*x*vz + w*w*vz

	return rotatedV
}

// Scale scales the quanternion by given scalar
func (q *Quaternion) Scale(s float64) {
	q.W = q.W * s
	q.X = q.X * s
	q.Y = q.Y * s
	q.Z = q.Z * s
}

// Random returns a random quaternion vector with unit length
func Random() Quaternion {
	s := rand.NewSource(time.Now().UnixNano())
	r := rand.New(s)
	v := [3]float64{r.Float64(), r.Float64(), r.Float64()}
	q := Quaternion{1, 0, 0, 0}
	q.W = math.Cos(2*math.Pi*v[0]) * math.Sqrt(v[2])
	q.X = math.Sin(2*math.Pi*v[1]) * math.Sqrt(1-v[2])
	q.Y = math.Cos(2*math.Pi*v[1]) * math.Sqrt(1-v[2])
	q.Z = math.Sin(2*math.Pi*v[0]) * math.Sqrt(v[2])

	return q
}

// FromAngleAxis returns a quaternion converted from angle axis pair
func FromAngleAxis(ang float64, axis [3]float64, indegree bool) Quaternion {
	if indegree {
		ang = ang / 180 * math.Pi
	}

	axisVectorLen := math.Sqrt(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2])

	q := Quaternion{1, 0, 0, 0}

	q.W = math.Cos(ang / 2)
	q.X = math.Sin(ang/2) * axis[0] / axisVectorLen
	q.Y = math.Sin(ang/2) * axis[1] / axisVectorLen
	q.Z = math.Sin(ang/2) * axis[2] / axisVectorLen

	return q
}

// FromBungeEulers returns a quaternion converted from given Bunge Euler angles
func FromBungeEulers(eulers [3]float64, indegree bool) Quaternion {
	// convert to radians for calculation
	if indegree {
		for i, v := range eulers {
			eulers[i] = v / 180 * math.Pi
		}
	}

	var c [3]float64
	var s [3]float64
	for i, v := range eulers {
		c[i] = math.Cos(v)
		s[i] = math.Sin(v)
	}

	return Quaternion{
		W: c[0]*c[1]*c[2] - s[0]*c[1]*s[2],
		X: c[0]*s[1]*c[2] + s[0]*s[1]*s[2],
		Y: -c[0]*s[1]*s[2] + s[0]*s[1]*c[2],
		Z: c[0]*c[1]*s[2] + s[0]*c[1]*c[2],
	}
}
