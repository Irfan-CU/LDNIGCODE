#include "mesh.h"


void Mesh::clear()
{
	vertex_hash_map.clear();
}
Point3 Mesh::min() const
{
	return aabb.min;
}
Point3 Mesh::max() const
{
	return aabb.max;
}
AABB3D Mesh::getAABB() const
{
	return aabb;
}