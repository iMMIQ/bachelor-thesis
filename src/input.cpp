#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include "input.h"

auto read_stl(const std::string &file) -> vector<Triangle> {
  Assimp::Importer importer;

  const aiScene *scene = importer.ReadFile(file, aiProcess_Triangulate);

  vector<Triangle> result;

  for (int i = 0; i < scene->mNumMeshes; ++i) {
    const aiMesh *mesh = scene->mMeshes[i];

    for (int j = 0; j < mesh->mNumFaces; ++j) {
      const aiFace &face = mesh->mFaces[j];

      const auto t = Triangle(Point3D(mesh->mVertices[face.mIndices[0]].x,
                                      mesh->mVertices[face.mIndices[0]].y,
                                      mesh->mVertices[face.mIndices[0]].z),
                              Point3D(mesh->mVertices[face.mIndices[1]].x,
                                      mesh->mVertices[face.mIndices[1]].y,
                                      mesh->mVertices[face.mIndices[1]].z),
                              Point3D(mesh->mVertices[face.mIndices[2]].x,
                                      mesh->mVertices[face.mIndices[2]].y,
                                      mesh->mVertices[face.mIndices[2]].z));

      result.push_back(t);
    }
  }

  return result;
}
