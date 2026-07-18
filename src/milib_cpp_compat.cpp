// mi-lib headers declare C++-only static class members (zVec3D::zvec3Dzero,
// zMat3D::zmat3Dident, ...) whose definitions normally come from the
// lib<name>_cpp.so variants — the same C sources rebuilt with g++. Those
// variants miscompile the roki-fd/zm ODE integration path (the stock
// roki-fd example heap-corrupts against them, zm 1.14.5), so rtctrl links
// the plain C libraries and defines the statics its translation units
// reference here. Constructors are header-inline, so these definitions
// depend on nothing from the cpp variants.
#include <zeo/zeo_frame3d.h>
#include <zeo/zeo_mat3d.h>
#include <zeo/zeo_vec3d.h>

const zVec3D zVec3D::zvec3Dzero(0.0, 0.0, 0.0);
const zVec3D zVec3D::zvec3Dx(1.0, 0.0, 0.0);
const zVec3D zVec3D::zvec3Dy(0.0, 1.0, 0.0);
const zVec3D zVec3D::zvec3Dz(0.0, 0.0, 1.0);

const zMat3D zMat3D::zmat3Dzero(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
const zMat3D zMat3D::zmat3Dident(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);

const zFrame3D zFrame3D::zframe3Dident(0.0, 0.0, 0.0,  //
                                       1.0, 0.0, 0.0,  //
                                       0.0, 1.0, 0.0,  //
                                       0.0, 0.0, 1.0);
