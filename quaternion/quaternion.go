/*
Quanternion calculation for manipulating cyxtal orientations
rotations.
*/

package quanternion

// Quternion: q = w + Xi + Yj + Zk
type Quanternion struct {
	W float64 // sin(theta/2)
	X float64 // vx
	Y float64 // vy
	Z float64 // vz
}
