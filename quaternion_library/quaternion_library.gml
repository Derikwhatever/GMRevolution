/// Quaternion scripts
/// 08/01/2023
/// @callmeEthan
/// Adapted by Derik.whatever to work with DragoniteSpam's Vector3 library.
function quat(x = 0.00001, y = 0, z = 0, w = 1) constructor {
    self.x = x;
    self.y = y;
    self.z = z;
	self.w = w;

	// Multiply two quaternion and combine two rotation together.
    static Mul = function(val) {
		var qx = self.w * val.x + self.x * val.w + self.y * val.z - self.z * val.y;
		var qy = self.w * val.y + self.y * val.w + self.z * val.x - self.x * val.z;
		var qz = self.w * val.z + self.z * val.w + self.x * val.y - self.y * val.x;
		var qw = self.w * val.w - self.x * val.x - self.y * val.y - self.z * val.z;
		return new quat(qx, qy, qz, qw);
	}

	// Rotate a vector by a quaternion angle
	// If an array is not supplied, return a new array.
	// https://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.html
	static TransformVec3 = function(vec) {
		var qx = self.w*self.w*vec.x + 2*self.w*self.w*vec.z - 2*self.z*self.w*vec.y + self.x*self.x*vec.x + 2*self.y*self.x*vec.y + 2*self.z*self.x*vec.z - self.z*self.z*vec.x - self.y*self.y*vec.x;
		var qy = 2*self.x*self.y*vec.x + self.y*self.y*vec.y + 2*self.z*self.y*vec.z + 2*self.w*self.z*vec.x - self.z*self.z*vec.y + self.w*self.w*vec.y - 2*self.x*self.w*vec.z - self.x*self.x*vec.y;
		var qz = 2*self.x*self.z*vec.x + 2*self.y*self.z*vec.y + self.z*self.z*vec.z - 2*self.w*self.y*vec.x - self.y*self.y*vec.z + 2*self.w*self.x*vec.y - self.x*self.x*vec.z + self.w*self.w*vec.z;
		return new vec3(qx, qy, qz);
	}

	// Rotate a quaternion around it's local axis.
	static RotateVec3 = function(vec) {
		var vec_quat = vec3toquat(vec);
		return self.Mul(vec_quat);
	}

	//	Return the conjugate of a quaternion 
	static Conjugate = function () {
	return new quat(-self.x, -self.y, -self.z, self.w);
	}

	//	Same thing as quaternion conjugate, in case you don't know...
	static Inverse = function () {
	return new self.Conjugate();
	}
	
}

//Convert rotation (radians) to quaternion angle
function vec3toquat(vec) {

    var cr = cos(vec.x * 0.5);
    var sr = sin(vec.x * 0.5);
    var cp = cos(vec.y * 0.5);
    var sp = sin(vec.y * 0.5);
    var cy = cos(vec.z * 0.5);
    var sy = sin(vec.z * 0.5);
	
    var qx = sr * cp * cy - cr * sp * sy;
    var qy = cr * sp * cy + sr * cp * sy;
    var qz = cr * cp * sy - sr * sp * cy;
    var qw = cr * cp * cy + sr * sp * sy;
	
	return new quat(qx, qy, qz, qw);
	
}

// Build transform matrix based on quaternion rotation instead of Euler angle
// https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm
function matrix_build_quat(_x, _y, _z, _quat, _xscale, _yscale, _zscale) {
	var mat = array_create(16,0);
   var sqw = _quat.w*_quat.w;
   var sqx = _quat.x*_quat.x;
   var sqy = _quat.y*_quat.y;
   var sqz = _quat.z*_quat.z;
   mat[@0] = (sqx - sqy - sqz + sqw) * _xscale; // since sqw + sqx + sqy + sqz =1
   mat[@5] = (-sqx + sqy - sqz + sqw) * _yscale;
   mat[@10] = (-sqx - sqy + sqz + sqw) * _zscale;
   
   var tmp1 = _quat.x*_quat.y;
   var tmp2 = _quat.z*_quat.w;
   mat[@1] = 2.0 * (tmp1 + tmp2) * _xscale;
   mat[@4] = 2.0 * (tmp1 - tmp2) * _yscale;
   
   tmp1 = _quat.x*_quat.z;
   tmp2 = _quat.y*_quat.w;
   mat[@2] = 2.0 * (tmp1 - tmp2) * _xscale;
   mat[@8] = 2.0 * (tmp1 + tmp2) * _zscale;
   
   tmp1 = _quat.y*_quat.z;
   tmp2 = _quat.x*_quat.w;
   mat[@6] = 2.0 * (tmp1 + tmp2) * _yscale;
   mat[@9] = 2.0 * (tmp1 - tmp2) * _zscale;
	
	mat[@12] = _x;
	mat[@13] = _y;
	mat[@14] = _z;
	mat[@15] = 1.0;
	
}
