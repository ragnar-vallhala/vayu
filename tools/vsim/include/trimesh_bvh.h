// trimesh_bvh.h — shared static-triangle-mesh collider for the world import.
//
// Header-only, dependency-free (only vsim_math.h). Used by BOTH sides:
//   - the GCS builds a BVH over an imported world mesh and serializes it to a
//     flat byte blob (written to an mmap'd file), via buildSerialized().
//   - vsim_d maps that blob read-only and traverses it each physics step via
//     Bvh::fromBytes() + querySphere() + closestPointOnTriangle().
//
// The serialized layout is one self-describing blob (explicit offsets, 4-byte
// fields, little-endian) so the daemon can mmap it and use it in place with no
// parsing. All geometry is in the NED world frame (the import transform is
// baked in GCS-side before building).
#ifndef VSIM_TRIMESH_BVH_H
#define VSIM_TRIMESH_BVH_H

#include "vsim_math.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

namespace vsim {
namespace trimesh {

constexpr uint32_t kMagic   = 0x444C5756u;  // 'VWLD'
constexpr uint32_t kVersion = 1u;
constexpr uint32_t kFlagDoubleSided = 1u;
constexpr int      kLeafTris = 8;

// Self-describing blob header. Regions follow in this order, each padded to a
// 16-byte boundary: vertices(float[3]) | triangles(uint32[3]) | tri_perm(uint32)
// | nodes(Node).
struct Header {
    uint32_t magic;
    uint32_t version;
    uint32_t vertex_count;
    uint32_t triangle_count;
    uint32_t node_count;
    uint32_t flags;
    float    aabb_min[3];
    float    aabb_max[3];
    uint64_t vertices_off;
    uint64_t triangles_off;
    uint64_t tri_perm_off;
    uint64_t nodes_off;
    uint64_t total_size;
};

// 32-byte cache-friendly node. Leaf: left == -1, [first_tri, first_tri+
// tri_count) indexes tri_perm. Interior: left = left-child index, first_tri =
// right-child index, tri_count == 0.
struct Node {
    float   bmin[3];
    float   bmax[3];
    int32_t left;
    int32_t first_tri;
    int32_t tri_count;
    int32_t pad;
};

// ---- small math helpers (Vec3 has no dot/length) ----
inline float dot(const Vec3& a, const Vec3& b) {
    return a.x() * b.x() + a.y() * b.y() + a.z() * b.z();
}
inline float length(const Vec3& a) { return std::sqrt(dot(a, a)); }

// Closest point on triangle abc to p (Ericson, Real-Time Collision Detection).
inline Vec3 closestPointOnTriangle(const Vec3& p, const Vec3& a, const Vec3& b,
                                   const Vec3& c) {
    const Vec3 ab = b - a, ac = c - a, ap = p - a;
    const float d1 = dot(ab, ap), d2 = dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f) return a;
    const Vec3 bp = p - b;
    const float d3 = dot(ab, bp), d4 = dot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3) return b;
    const float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        const float v = d1 / (d1 - d3);
        return a + ab * v;
    }
    const Vec3 cp = p - c;
    const float d5 = dot(ab, cp), d6 = dot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6) return c;
    const float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        const float w = d2 / (d2 - d6);
        return a + ac * w;
    }
    const float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + (c - b) * w;
    }
    const float denom = 1.0f / (va + vb + vc);
    const float v = vb * denom, w = vc * denom;
    return a + ab * v + ac * w;
}

// ================= Builder (GCS / host side) =================
namespace detail {
struct BuildTri { float cmin[3], cmax[3], centroid[3]; uint32_t idx; };

inline void triBounds(const float* verts, const uint32_t* tris, uint32_t t,
                      BuildTri& bt) {
    const uint32_t* tri = tris + 3 * t;
    for (int k = 0; k < 3; ++k) {
        float lo = +1e30f, hi = -1e30f, c = 0.0f;
        for (int j = 0; j < 3; ++j) {
            const float v = verts[3 * tri[j] + k];
            lo = std::min(lo, v); hi = std::max(hi, v); c += v;
        }
        bt.cmin[k] = lo; bt.cmax[k] = hi; bt.centroid[k] = c / 3.0f;
    }
    bt.idx = t;
}

// Recursive median build. Returns the emitted node index.
inline int32_t build(std::vector<Node>& nodes, std::vector<BuildTri>& bt,
                     int start, int end) {
    const int32_t self = static_cast<int32_t>(nodes.size());
    nodes.emplace_back();
    Node n{};
    float bmin[3] = {+1e30f, +1e30f, +1e30f}, bmax[3] = {-1e30f, -1e30f, -1e30f};
    float cmin[3] = {+1e30f, +1e30f, +1e30f}, cmax[3] = {-1e30f, -1e30f, -1e30f};
    for (int i = start; i < end; ++i)
        for (int k = 0; k < 3; ++k) {
            bmin[k] = std::min(bmin[k], bt[i].cmin[k]);
            bmax[k] = std::max(bmax[k], bt[i].cmax[k]);
            cmin[k] = std::min(cmin[k], bt[i].centroid[k]);
            cmax[k] = std::max(cmax[k], bt[i].centroid[k]);
        }
    for (int k = 0; k < 3; ++k) { n.bmin[k] = bmin[k]; n.bmax[k] = bmax[k]; }

    const int count = end - start;
    if (count <= kLeafTris) {
        n.left = -1; n.first_tri = start; n.tri_count = count;
        nodes[self] = n;
        return self;
    }
    int axis = 0;
    float ext = cmax[0] - cmin[0];
    if (cmax[1] - cmin[1] > ext) { axis = 1; ext = cmax[1] - cmin[1]; }
    if (cmax[2] - cmin[2] > ext) { axis = 2; }
    const int mid = (start + end) / 2;
    std::nth_element(bt.begin() + start, bt.begin() + mid, bt.begin() + end,
                     [axis](const BuildTri& a, const BuildTri& b) {
                         return a.centroid[axis] < b.centroid[axis];
                     });
    n.tri_count = 0;
    nodes[self] = n;  // reserve slot before recursing (vector may realloc)
    const int32_t l = build(nodes, bt, start, mid);
    const int32_t r = build(nodes, bt, mid, end);
    nodes[self].left = l;
    nodes[self].first_tri = r;
    return self;
}
}  // namespace detail

// Build a serialized BVH blob over the given vertex/triangle arrays.
inline std::vector<uint8_t> buildSerialized(const float* verts, uint32_t nverts,
                                            const uint32_t* tris, uint32_t ntris,
                                            bool doubleSided) {
    std::vector<Node> nodes;
    std::vector<uint32_t> perm(ntris);
    if (ntris > 0) {
        std::vector<detail::BuildTri> bt(ntris);
        for (uint32_t t = 0; t < ntris; ++t) detail::triBounds(verts, tris, t, bt[t]);
        nodes.reserve(2 * ntris);
        detail::build(nodes, bt, 0, static_cast<int>(ntris));
        for (uint32_t i = 0; i < ntris; ++i) perm[i] = bt[i].idx;
    } else {
        Node n{};
        n.left = -1; n.first_tri = 0; n.tri_count = 0;
        nodes.push_back(n);
    }

    auto align16 = [](uint64_t v) { return (v + 15u) & ~uint64_t(15); };
    const uint64_t vOff = align16(sizeof(Header));
    const uint64_t tOff = align16(vOff + uint64_t(nverts) * 3 * sizeof(float));
    const uint64_t pOff = align16(tOff + uint64_t(ntris) * 3 * sizeof(uint32_t));
    const uint64_t nOff = align16(pOff + uint64_t(ntris) * sizeof(uint32_t));
    const uint64_t total = align16(nOff + nodes.size() * sizeof(Node));

    std::vector<uint8_t> blob(total, 0);
    Header h{};
    h.magic = kMagic; h.version = kVersion;
    h.vertex_count = nverts; h.triangle_count = ntris;
    h.node_count = static_cast<uint32_t>(nodes.size());
    h.flags = doubleSided ? kFlagDoubleSided : 0u;
    for (int k = 0; k < 3; ++k) { h.aabb_min[k] = nodes[0].bmin[k]; h.aabb_max[k] = nodes[0].bmax[k]; }
    h.vertices_off = vOff; h.triangles_off = tOff; h.tri_perm_off = pOff;
    h.nodes_off = nOff; h.total_size = total;
    std::memcpy(blob.data(), &h, sizeof(h));
    if (nverts) std::memcpy(blob.data() + vOff, verts, uint64_t(nverts) * 3 * sizeof(float));
    if (ntris)  std::memcpy(blob.data() + tOff, tris, uint64_t(ntris) * 3 * sizeof(uint32_t));
    if (ntris)  std::memcpy(blob.data() + pOff, perm.data(), uint64_t(ntris) * sizeof(uint32_t));
    std::memcpy(blob.data() + nOff, nodes.data(), nodes.size() * sizeof(Node));
    return blob;
}

// ================= Query view (daemon side) =================
struct Bvh {
    const Header*   h     = nullptr;
    const float*    verts = nullptr;
    const uint32_t* tris  = nullptr;
    const uint32_t* perm  = nullptr;
    const Node*     nodes = nullptr;

    bool valid() const { return h != nullptr && h->triangle_count > 0; }
    bool doubleSided() const { return h && (h->flags & kFlagDoubleSided); }
    Vec3 vert(uint32_t i) const {
        return Vec3(verts[3 * i], verts[3 * i + 1], verts[3 * i + 2]);
    }
    void tri(uint32_t t, Vec3& a, Vec3& b, Vec3& c) const {
        const uint32_t* f = tris + 3 * t;
        a = vert(f[0]); b = vert(f[1]); c = vert(f[2]);
    }

    static Bvh fromBytes(const uint8_t* base, size_t len) {
        Bvh r;
        if (!base || len < sizeof(Header)) return r;
        const Header* h = reinterpret_cast<const Header*>(base);
        if (h->magic != kMagic || h->version != kVersion) return r;
        if (h->total_size > len) return r;
        if (h->nodes_off + uint64_t(h->node_count) * sizeof(Node) > len) return r;
        r.h     = h;
        r.verts = reinterpret_cast<const float*>(base + h->vertices_off);
        r.tris  = reinterpret_cast<const uint32_t*>(base + h->triangles_off);
        r.perm  = reinterpret_cast<const uint32_t*>(base + h->tri_perm_off);
        r.nodes = reinterpret_cast<const Node*>(base + h->nodes_off);
        return r;
    }
};

// Collect triangle indices in leaves whose AABB overlaps sphere(c, r). Writes
// up to `cap` indices into `out`; returns the count (clamped to cap). Fixed
// traversal stack, no allocation — safe in the 8 kHz loop.
inline int querySphere(const Bvh& b, const Vec3& c, float r, uint32_t* out,
                       int cap) {
    if (!b.valid() || cap <= 0) return 0;
    const float r2 = r * r;
    auto overlaps = [&](const Node& n) {
        float d2 = 0.0f;
        const float p[3] = {c.x(), c.y(), c.z()};
        for (int k = 0; k < 3; ++k) {
            const float e = (p[k] < n.bmin[k]) ? n.bmin[k] - p[k]
                          : (p[k] > n.bmax[k]) ? p[k] - n.bmax[k] : 0.0f;
            d2 += e * e;
        }
        return d2 <= r2;
    };
    int32_t stack[64];
    int sp = 0, n = 0;
    stack[sp++] = 0;
    while (sp > 0) {
        const Node& nd = b.nodes[stack[--sp]];
        if (!overlaps(nd)) continue;
        if (nd.left < 0) {  // leaf
            for (int i = 0; i < nd.tri_count && n < cap; ++i)
                out[n++] = b.perm[nd.first_tri + i];
            if (n >= cap) break;
        } else if (sp + 2 <= 64) {
            stack[sp++] = nd.left;
            stack[sp++] = nd.first_tri;  // right child
        }
    }
    return n;
}

}  // namespace trimesh
}  // namespace vsim

#endif  // VSIM_TRIMESH_BVH_H
