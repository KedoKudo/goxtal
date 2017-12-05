/*
orientation calculation for manipulating cyxtal orientations
rotations.
*/

package orientation

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

// Normalized  normalize quanternion norm to 1 (necessary for rotation)
func (q *Quaternion) Normalized() {
	qnorm := q.Norm()

	q.W = q.W / qnorm
	q.X = q.X / qnorm
	q.Y = q.Y / qnorm
	q.Z = q.Z / qnorm

}

// Normalize returns the normalized quaternion
func (q *Quaternion) Normalize() Quaternion {
	qnorm := q.Norm()

	return Quaternion{
		W: q.W / qnorm,
		X: q.X / qnorm,
		Y: q.Y / qnorm,
		Z: q.Z / qnorm,
	}
}

// Conjugated convert the quaternion to its conjugate
func (q *Quaternion) Conjugated() {
	q.X = -q.X
	q.Y = -q.Y
	q.Z = -q.Z
}

// Conjugate returns the conjugate of the quaternion
func (q *Quaternion) Conjugate() Quaternion {
	return Quaternion{
		W: q.W,
		X: -q.X,
		Y: -q.Y,
		Z: -q.Z,
	}
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

// Diff returns the total difference between two quaternions
func (q *Quaternion) Diff(q2 Quaternion) float64 {
	delta1 := math.Abs(q.W - q2.W + q.X - q2.X + q.Y - q2.Y + q.Z - q2.Z)
	delta2 := math.Abs(-q.W - q2.W - q.X - q2.X - q.Y - q2.Y - q.Z - q2.Z)

	if delta1 < delta2 {
		return delta1
	}

	return delta2
}

// AsArray returns the quaternion as a simple float64 array
func (q *Quaternion) AsArray() [4]float64 {
	return [4]float64{q.W, q.X, q.Y, q.Z}
}

// AsBungeEulers returns the Euler angles equivalent of the quaternion
// Orientation as Bunge-Euler angles.
//       Conversion of ACTIVE rotation to Euler angles taken from:
//       Melcher, A.; Unser, A.; Reichhardt, M.; Nestler, B.; Poetschke, M.; Selzer, M.
//       Conversion of EBSD data by a quaternion based algorithm to be used for grain structure simulations
//       Technische Mechanik 30 (2010) pp 401--413.
func (q *Quaternion) AsBungeEulers(indegree bool) [3]float64 {
	eulers := [3]float64{0, 0, 0}

	if (math.Abs(q.X) < 1e-10) && (math.Abs(q.Y) < 1e-10) {
		cosphi1 := q.W*q.W - q.Z*q.Z
		sinphi1 := 2 * q.W * q.Z
		eulers[0] = math.Atan2(sinphi1, cosphi1)
	} else if (math.Abs(q.W) < 1e-10) && (math.Abs(q.Z) < 1e-10) {
		cosphi1 := q.X*q.X - q.Y*q.Y
		sinphi1 := 2 * q.X * q.Y
		eulers[0] = math.Atan2(sinphi1, cosphi1)

		eulers[1] = math.Pi // PHI = pi
	} else {
		chi := math.Sqrt((q.W*q.W + q.Z*q.Z) * (q.X*q.X + q.Y*q.Y))

		x := (q.W*q.X - q.Y*q.Z) / 2. / chi
		y := (q.W*q.Y + q.X*q.Z) / 2. / chi
		eulers[0] = math.Atan2(y, x)

		x = q.W*q.W + q.Z*q.Z - (q.X*q.X + q.Y*q.Y)
		y = 2 * chi
		eulers[1] = math.Atan2(y, x)

		x = (q.W*q.X + q.Y*q.Z) / 2. / chi
		y = (q.Z*q.X - q.Y*q.W) / 2. / chi
		eulers[2] = math.Atan2(y, x)
	}

	if indegree {
		for i, v := range eulers {
			eulers[i] = v / math.Pi * 180
		}
	}

	return eulers
}

// FromMatrix set the quaternion with given rotation matrix
// Modified Method to calculate Quaternion from Orientation Matrix,
// Source: http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
func (q *Quaternion) FromMatrix(r [3][3]float64) {
	tr := r[0][0] + r[1][1] + r[2][2]
	s := math.Sqrt(tr+1) * 2.0

	if math.Abs(tr) > 1e-8 {
		q.W = s * 0.25
		q.X = (r[2][1] - r[1][2]) / s
		q.Y = (r[0][2] - r[2][0]) / s
		q.Z = (r[1][0] - r[0][1]) / s
	} else if (r[0][0] > r[1][1]) && (r[0][0] > r[2][2]) {
		t := r[0][0] - r[1][1] - r[2][2] + 1.0
		s = 2.0 * math.Sqrt(t)

		q.W = (r[2][1] - r[1][2]) / s
		q.X = s * 0.25
		q.Y = (r[0][1] + r[1][0]) / s
		q.Z = (r[2][0] + r[0][2]) / s
	} else if r[1][1] > r[2][2] {
		t := -r[0][0] + r[1][1] - r[2][2] + 1.0
		s = 2.0 * math.Sqrt(t)

		q.W = (r[0][2] - r[2][0]) / s
		q.X = (r[0][1] + r[1][0]) / s
		q.Y = s * 0.25
		q.Z = (r[1][2] + r[2][1]) / s
	} else {
		t := -r[0][0] - r[1][1] + r[2][2] + 1.0
		s = 2.0 * math.Sqrt(t)

		q.W = (r[1][0] - r[0][1]) / s
		q.X = (r[2][0] + r[0][2]) / s
		q.Y = (r[1][2] + r[2][1]) / s
		q.Z = s * 0.25
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

// ScaledBy scales the quanternion by given scalar
func (q *Quaternion) ScaledBy(s float64) {
	q.W = q.W * s
	q.X = q.X * s
	q.Y = q.Y * s
	q.Z = q.Z * s
}

// Random make a random quaternion vector with unit length
func (q *Quaternion) Random() {
	s := rand.NewSource(time.Now().UnixNano())
	r := rand.New(s)
	v := [3]float64{r.Float64(), r.Float64(), r.Float64()}

	q.W = math.Cos(2*math.Pi*v[0]) * math.Sqrt(v[2])
	q.X = math.Sin(2*math.Pi*v[1]) * math.Sqrt(1-v[2])
	q.Y = math.Cos(2*math.Pi*v[1]) * math.Sqrt(1-v[2])
	q.Z = math.Sin(2*math.Pi*v[0]) * math.Sqrt(v[2])
}

// FromAngleAxis returns a quaternion converted from angle axis pair
func (q *Quaternion) FromAngleAxis(ang float64, axis [3]float64, indegree bool) {
	if indegree {
		ang = ang / 180 * math.Pi
	}

	axisVectorLen := math.Sqrt(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2])

	q.W = math.Cos(ang / 2)
	q.X = math.Sin(ang/2) * axis[0] / axisVectorLen
	q.Y = math.Sin(ang/2) * axis[1] / axisVectorLen
	q.Z = math.Sin(ang/2) * axis[2] / axisVectorLen
}

// AsAngleAxis returns the angle-axis pair representation of the orientation
func (q *Quaternion) AsAngleAxis(indegree bool) (ang float64, axis [3]float64) {
	ang = math.Acos(q.W) * 2

	norm := math.Sqrt(q.X*q.X + q.Y*q.Y + q.Z*q.Z)
	axis = [3]float64{q.X / norm, q.Y / norm, q.Z / norm}

	if indegree {
		ang = ang / math.Pi * 180
	}

	return ang, axis
}

// FromBungeEulers returns a quaternion converted from given Bunge Euler angles
func (q *Quaternion) FromBungeEulers(eulers [3]float64, indegree bool) {
	// convert to radians for calculation
	if indegree {
		for i, v := range eulers {
			eulers[i] = v / 180 * math.Pi
		}
	}

	var c [3]float64
	var s [3]float64
	for i, v := range eulers {
		c[i] = math.Cos(v / 2)
		s[i] = math.Sin(v / 2)
	}

	q.W = c[0]*c[1]*c[2] - s[0]*c[1]*s[2]
	q.X = c[0]*s[1]*c[2] + s[0]*s[1]*s[2]
	q.Y = -c[0]*s[1]*s[2] + s[0]*s[1]*c[2]
	q.Z = c[0]*c[1]*s[2] + s[0]*c[1]*c[2]
}
