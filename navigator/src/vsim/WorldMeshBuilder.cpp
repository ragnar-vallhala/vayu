// WorldMeshBuilder.cpp — see header. This TU includes trimesh_bvh.h (and thus
// vsim_math.h's struct Vec3) and must NOT include VsimTypes.h / Qt headers, so
// the two conflicting `vsim::Vec3` definitions never meet.
#include "WorldMeshBuilder.h"

#include "trimesh_bvh.h"

namespace vsim {

std::vector<uint8_t> buildWorldBvh(const float* verts, uint32_t nverts,
                                   bool doubleSided, uint32_t& outNodeCount) {
    outNodeCount = 0;
    if (!verts || nverts < 3) return {};
    const uint32_t ntris = nverts / 3;
    // Soup: triangle k uses vertices 3k, 3k+1, 3k+2 (no welding for v1).
    std::vector<uint32_t> tris(static_cast<size_t>(ntris) * 3);
    for (uint32_t i = 0; i < ntris * 3; ++i) tris[i] = i;
    std::vector<uint8_t> blob =
        trimesh::buildSerialized(verts, nverts, tris.data(), ntris, doubleSided);
    trimesh::Bvh bvh = trimesh::Bvh::fromBytes(blob.data(), blob.size());
    if (bvh.valid()) outNodeCount = bvh.h->node_count;
    return blob;
}

}  // namespace vsim
