#pragma once

// Motion control module: shared data structures for rigid-object motion,
// skeletal (skinned) human gaussian-splatting motion, and facial animation.
//
// This header is included from both host (.cpp/.h) and device (.cu) translation
// units, so it must stay dependency-light: only glm core (mat/vec) and the
// standard library. Quaternion math is implemented locally with scalar
// std::sin/std::cos/etc. because the vendored GLM version's gtc/quaternion
// helpers do not compile cleanly with scalar arguments.

#include <cmath>
#include <vector>
#include <string>
#include <cstdint>
#include <cstring>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp> // glm::quat type definition (we do not call its functions)

using glm::vec3;
using glm::vec4;
using glm::mat3;
using glm::mat4;
// Note: glm::quat is used as a plain value type (x,y,z,w storage). We avoid
// glm's quaternion *functions* and implement the math we need below.
using glm::quat;

namespace motion {

using NodeID = int64_t;

// --- Minimal local quaternion math (scalar std:: functions) ---------------

inline quat quatIdentity() { return quat(1.0f, 0.0f, 0.0f, 0.0f); } // (w, x, y, z)

inline float quatDot(const quat& a, const quat& b) {
	return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
}

inline quat quatNormalize(const quat& q) {
	float n = std::sqrt(quatDot(q, q));
	if(n <= 0.0f) return quatIdentity();
	float inv = 1.0f / n;
	return quat(q.w * inv, q.x * inv, q.y * inv, q.z * inv);
}

inline quat quatConjugate(const quat& q) { return quat(q.w, -q.x, -q.y, -q.z); }

// Compose two rotations: result = a then b (b applied in a's frame), i.e.
// standard glm semantics where `a * b` applies b first.
inline quat quatMul(const quat& a, const quat& b) {
	return quat(
		a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, // w
		a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, // x
		a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, // y
		a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w  // z
	);
}

inline quat operator*(const quat& a, const quat& b) { return quatMul(a, b); }

// Convert a quaternion to a rotation matrix.
inline mat4 quatToMat4(const quat& q) {
	quat n = quatNormalize(q);
	float xx = n.x * n.x, yy = n.y * n.y, zz = n.z * n.z;
	float xy = n.x * n.y, xz = n.x * n.z, yz = n.y * n.z;
	float wx = n.w * n.x, wy = n.w * n.y, wz = n.w * n.z;

	mat4 m(1.0f);
	m[0][0] = 1.0f - 2.0f * (yy + zz);
	m[0][1] = 2.0f * (xy + wz);
	m[0][2] = 2.0f * (xz - wy);
	m[1][0] = 2.0f * (xy - wz);
	m[1][1] = 1.0f - 2.0f * (xx + zz);
	m[1][2] = 2.0f * (yz + wx);
	m[2][0] = 2.0f * (xz + wy);
	m[2][1] = 2.0f * (yz - wx);
	m[2][2] = 1.0f - 2.0f * (xx + yy);
	return m;
}

// Quaternion from an axis (assumed normalized) and angle in radians.
inline quat quatAxisAngle(const vec3& axis, float angle) {
	float h = 0.5f * angle;
	float s = std::sin(h);
	return quat(std::cos(h), axis.x * s, axis.y * s, axis.z * s);
}

// Spherical linear interpolation.
inline quat slerpQuat(const quat& a, const quat& b, float t) {
	quat z = b;
	float cosTheta = quatDot(a, z);
	if(cosTheta < 0.0f) {
		z = quat(-z.w, -z.x, -z.y, -z.z);
		cosTheta = -cosTheta;
	}
	const float threshold = 0.9995f;
	if(cosTheta > threshold) {
		// Near-parallel: linear interpolation, then normalize.
		quat r(
			(1.0f - t) * a.w + t * z.w,
			(1.0f - t) * a.x + t * z.x,
			(1.0f - t) * a.y + t * z.y,
			(1.0f - t) * a.z + t * z.z);
		return quatNormalize(r);
	}
	float angle = std::acos(cosTheta);
	float sinAngle = std::sin(angle);
	float s0 = std::sin((1.0f - t) * angle) / sinAngle;
	float s1 = std::sin(t * angle) / sinAngle;
	return quat(
		s0 * a.w + s1 * z.w,
		s0 * a.x + s1 * z.x,
		s0 * a.y + s1 * z.y,
		s0 * a.z + s1 * z.z);
}

// Extract a quaternion from the rotation part of a (row-major glm) matrix.
inline quat quatFromMat4(const mat4& m) {
	float trace = m[0][0] + m[1][1] + m[2][2];
	quat q;
	if(trace > 0.0f) {
		float s = 0.5f / std::sqrt(trace + 1.0f);
		q.w = 0.25f / s;
		q.x = (m[1][2] - m[2][1]) * s;
		q.y = (m[2][0] - m[0][2]) * s;
		q.z = (m[0][1] - m[1][0]) * s;
	} else if(m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
		float s = 2.0f * std::sqrt(1.0f + m[0][0] - m[1][1] - m[2][2]);
		q.w = (m[1][2] - m[2][1]) / s;
		q.x = 0.25f * s;
		q.y = (m[1][0] + m[0][1]) / s;
		q.z = (m[2][0] + m[0][2]) / s;
	} else if(m[1][1] > m[2][2]) {
		float s = 2.0f * std::sqrt(1.0f + m[1][1] - m[0][0] - m[2][2]);
		q.w = (m[2][0] - m[0][2]) / s;
		q.x = (m[1][0] + m[0][1]) / s;
		q.y = 0.25f * s;
		q.z = (m[2][1] + m[1][2]) / s;
	} else {
		float s = 2.0f * std::sqrt(1.0f + m[2][2] - m[0][0] - m[1][1]);
		q.w = (m[0][1] - m[1][0]) / s;
		q.x = (m[2][0] + m[0][2]) / s;
		q.y = (m[2][1] + m[1][2]) / s;
		q.z = 0.25f * s;
	}
	return quatNormalize(q);
}

// --- Motion data structures ----------------------------------------------

// A sampled rigid transform, decomposed into translation / rotation / scale.
struct TransformSample {
	vec3 translation = vec3(0.0f);
	quat rotation = quatIdentity();
	vec3 scale = vec3(1.0f);

	static TransformSample identity() { return TransformSample{}; }

	// Compose into a 4x4 local transform matrix (T * R * S).
	mat4 toMatrix() const {
		mat4 t = glm::translate(mat4(1.0f), translation);
		mat4 r = quatToMat4(rotation);
		mat4 s = glm::scale(mat4(1.0f), scale);
		return t * r * s;
	}

	// Decompose a matrix into a TransformSample. Implemented locally rather than
	// via glm::decompose: the vendored GLM version's quaternion helpers
	// (normalize/length) call scalar glm::sqrt which is not declared in this
	// build, and decompose would instantiate them on the host side.
	static TransformSample fromMatrix(const mat4& m) {
		TransformSample out;
		out.translation = vec3(m[3]); // translation column

		// Extract scale as the length of the three basis column vectors.
		vec3 c0(m[0][0], m[0][1], m[0][2]);
		vec3 c1(m[1][0], m[1][1], m[1][2]);
		vec3 c2(m[2][0], m[2][1], m[2][2]);
		float sx = std::sqrt(c0.x * c0.x + c0.y * c0.y + c0.z * c0.z);
		float sy = std::sqrt(c1.x * c1.x + c1.y * c1.y + c1.z * c1.z);
		float sz = std::sqrt(c2.x * c2.x + c2.y * c2.y + c2.z * c2.z);
		out.scale = vec3(sx, sy, sz);

		// Build a pure rotation matrix (columns normalized, sign-corrected).
		mat4 r(1.0f);
		r[0][0] = c0.x / sx; r[0][1] = c0.y / sx; r[0][2] = c0.z / sx;
		r[1][0] = c1.x / sy; r[1][1] = c1.y / sy; r[1][2] = c1.z / sy;
		r[2][0] = c2.x / sz; r[2][1] = c2.y / sz; r[2][2] = c2.z / sz;
		out.rotation = quatFromMat4(r);
		return out;
	}
};

// Local-space pose of a single joint (relative to its parent joint).
struct JointPose {
	quat rotation = quatIdentity();
	vec3 translation = vec3(0.0f);
	vec3 scale = vec3(1.0f);

	mat4 localMatrix() const {
		return TransformSample{translation, rotation, scale}.toMatrix();
	}
};

// A whole-skeleton pose: one JointPose per joint, indexed identically to
// Skeleton::jointNames / Skeleton::parents.
struct SkeletonPose {
	std::vector<JointPose> joints;

	void reset(size_t jointCount) {
		joints.assign(jointCount, JointPose{});
	}
};

// ARKit-compatible facial blendshape set: 52 coefficients in [0,1].
constexpr int BLENDSHAPE_COUNT = 52;

struct FaceData {
	float weights[BLENDSHAPE_COUNT];

	FaceData() { std::memset(weights, 0, sizeof(weights)); }

	void clear() { std::memset(weights, 0, sizeof(weights)); }
};

// Easing functions used by animated transitions and timeline interpolation.
enum class EaseMode {
	Linear,
	EaseIn,
	EaseOut,
	EaseInOut,
};

inline float applyEase(float u, EaseMode mode) {
	switch(mode) {
		case EaseMode::Linear:    return u;
		case EaseMode::EaseIn:    return u * u;
		case EaseMode::EaseOut:   return 1.0f - (1.0f - u) * (1.0f - u);
		case EaseMode::EaseInOut: return u < 0.5f ? 2.0f * u * u
		                                           : 1.0f - 0.5f * (1.0f - u) * (1.0f - u);
	}
	return u;
}

// Linear interpolation helpers (avoid relying on glm::mix scalar availability).
inline float mix(float a, float b, float t) { return a + (b - a) * t; }
inline vec3 mix(const vec3& a, const vec3& b, float t) { return a + (b - a) * t; }

// Quaternion to Euler angles (radians), XYZ rotation order. Used by the GUI to
// expose rotation in editable degree fields.
inline vec3 quatToEulerXYZ(const quat& q) {
	quat n = quatNormalize(q);
	// Pitch (x)
	float sinp = 2.0f * (n.w * n.x + n.y * n.z);
	float cosp = 1.0f - 2.0f * (n.x * n.x + n.y * n.y);
	float pitch = std::atan2(sinp, cosp);
	// Yaw (y)
	float sinYaw = 2.0f * (n.w * n.y - n.z * n.x);
	float yaw = (std::fabs(sinYaw) >= 1.0f) ? std::copysign(3.14159265358979f / 2.0f, sinYaw)
	                                       : std::asin(sinYaw);
	// Roll (z)
	float sinRoll = 2.0f * (n.w * n.z + n.x * n.y);
	float cosRoll = 1.0f - 2.0f * (n.y * n.y + n.z * n.z);
	float roll = std::atan2(sinRoll, cosRoll);
	return vec3(pitch, yaw, roll);
}

// Euler angles (radians, XYZ order) to quaternion.
inline quat quatFromEulerXYZ(const vec3& e) {
	float cx = std::cos(e.x * 0.5f), sx = std::sin(e.x * 0.5f);
	float cy = std::cos(e.y * 0.5f), sy = std::sin(e.y * 0.5f);
	float cz = std::cos(e.z * 0.5f), sz = std::sin(e.z * 0.5f);
	return quat(
		cx * cy * cz + sx * sy * sz, // w
		sx * cy * cz - cx * sy * sz, // x
		cx * sy * cz + sx * cy * sz, // y
		cx * cy * sz - sx * sy * cz  // z
	);
}

} // namespace motion
