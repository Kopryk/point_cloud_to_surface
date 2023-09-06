#pragma once

struct Vector3 {
	double x, y, z;

	Vector3 operator+(const Vector3& other) const {
		return Vector3{ x + other.x, y + other.y, z + other.z };
	}

	Vector3 operator-(const Vector3& other) const {
		return Vector3{ x - other.x, y - other.y, z - other.z };
	}

	Vector3 operator*(double scalar) const {
		return Vector3{ x * scalar, y * scalar, z * scalar };
	}

};


inline Vector3 operator*(double scalar, const Vector3& vec) {
	return vec * scalar;
}


class Triangle {

public:

	Triangle() = default;

	Triangle(const Vector3& v1, const Vector3& v2, const Vector3& v3) {
		vertices[0] = v1;
		vertices[1] = v2;
		vertices[2] = v3;
	}

	Vector3 getVertex(int index) const {
		if (index < 0 || index >= 3) {
			return Vector3();
		}
		return vertices[index];
	}


	void setVertex(int index, const Vector3& vertex) {
		if (index < 0 || index >= 3) {
			return;
		}
		vertices[index] = vertex;
	}

	Triangle operator-(const Triangle& other) const {
		Vector3 resultVertices[3];
		for (int i = 0; i < 3; i++) {
			resultVertices[i].x = this->vertices[i].x - other.vertices[i].x;
			resultVertices[i].y = this->vertices[i].y - other.vertices[i].y;
			resultVertices[i].z = this->vertices[i].z - other.vertices[i].z;
		}
		return Triangle(resultVertices[0], resultVertices[1], resultVertices[2]);
	}



	Vector3 vertices[3];
};
