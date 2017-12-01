/*
Quanternion calculation for manipulating cyxtal orientations
rotations.
*/

package quanternion

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

// RandomQuaternion returns a random quaternion vector with unit length
func RandomQuaternion() Quaternion {
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

// QuaternionFromAngleAxis returns a quaternion converted from angle axis pair
func QuaternionFromAngleAxis(ang float64, axis [3]float64, indegree bool) Quaternion {
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
