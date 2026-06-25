// trimesh_bvh_test.cpp — standalone (no Qt/daemon) check of the world collider.
//   g++ -std=c++17 -I tools/vsim/include tools/vsim/tests/trimesh_bvh_test.cpp -o /tmp/t && /tmp/t
#include "trimesh_bvh.h"

#include <cmath>
#include <cstdio>
#include <vector>

using namespace vsim;
using namespace vsim::trimesh;

static int g_checks = 0, g_fails = 0;
#define CHECK(cond, msg)                                                       \
    do {                                                                       \
        ++g_checks;                                                            \
        if (cond) printf("  ok   %s\n", (msg));                                \
        else { ++g_fails; printf("  FAIL %s (%s:%d)\n", (msg), __FILE__, __LINE__); } \
    } while (0)

static bool near(const Vec3& a, const Vec3& b, float e = 1e-4f) {
    return length(a - b) < e;
}

int main() {
    printf("== trimesh_bvh test ==\n");

    // --- closestPointOnTriangle: face / edge / vertex regions ---
    {
        Vec3 a(0, 0, 0), b(2, 0, 0), c(0, 2, 0);
        CHECK(near(closestPointOnTriangle(Vec3(0.5f, 0.5f, 5), a, b, c),
                   Vec3(0.5f, 0.5f, 0)), "closest: above-face projects onto face");
        CHECK(near(closestPointOnTriangle(Vec3(-1, -1, 0), a, b, c), a),
              "closest: vertex region -> a");
        CHECK(near(closestPointOnTriangle(Vec3(1, -1, 0), a, b, c), Vec3(1, 0, 0)),
              "closest: edge region -> edge ab");
    }

    // --- build a 10x10 floor (z=0) as a grid of triangles + serialize ---
    std::vector<float> verts;
    std::vector<uint32_t> tris;
    const int N = 10;                 // 10x10 cells -> 200 triangles
    auto vid = [&](int i, int j) { return uint32_t((N + 1) * i + j); };
    for (int i = 0; i <= N; ++i)
        for (int j = 0; j <= N; ++j) {
            verts.push_back(float(i) - 5.0f);  // x in [-5,5]
            verts.push_back(float(j) - 5.0f);  // y in [-5,5]
            verts.push_back(0.0f);             // z = 0 floor
        }
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j) {
            tris.push_back(vid(i, j)); tris.push_back(vid(i + 1, j)); tris.push_back(vid(i + 1, j + 1));
            tris.push_back(vid(i, j)); tris.push_back(vid(i + 1, j + 1)); tris.push_back(vid(i, j + 1));
        }
    const uint32_t nverts = uint32_t(verts.size() / 3), ntris = uint32_t(tris.size() / 3);

    std::vector<uint8_t> blob =
        buildSerialized(verts.data(), nverts, tris.data(), ntris, true);
    Bvh bvh = Bvh::fromBytes(blob.data(), blob.size());
    CHECK(bvh.valid(), "blob round-trips to a valid Bvh");
    CHECK(bvh.h->triangle_count == ntris, "triangle_count preserved");
    CHECK(bvh.h->vertex_count == nverts, "vertex_count preserved");
    CHECK(bvh.doubleSided(), "double-sided flag preserved");
    CHECK(std::fabs(bvh.h->aabb_min[0] + 5.0f) < 1e-4f &&
          std::fabs(bvh.h->aabb_max[0] - 5.0f) < 1e-4f, "root AABB spans the floor");

    // --- querySphere near a floor point returns local tris, not far ones ---
    uint32_t hits[256];
    int n = querySphere(bvh, Vec3(0, 0, 0.05f), 0.2f, hits, 256);
    printf("  (query at center returned %d candidate tris of %u)\n", n, ntris);
    CHECK(n > 0 && n < int(ntris), "query at floor center returns a localized subset");
    // every returned tri's closest point should be ~at the query's foot (z=0)
    bool all_close = true;
    for (int k = 0; k < n; ++k) {
        Vec3 a, b, c; bvh.tri(hits[k], a, b, c);
        Vec3 cp = closestPointOnTriangle(Vec3(0, 0, 0.05f), a, b, c);
        if (std::fabs(cp.z()) > 1e-4f) all_close = false;
    }
    CHECK(all_close, "returned tris are on the z=0 floor");

    // a point far above the floor and outside the skin returns nothing
    CHECK(querySphere(bvh, Vec3(0, 0, 3.0f), 0.2f, hits, 256) == 0,
          "query 3 m above floor returns no tris");
    // a point off the side of the floor returns nothing
    CHECK(querySphere(bvh, Vec3(20, 0, 0.05f), 0.2f, hits, 256) == 0,
          "query off the floor edge returns no tris");

    printf("\n%d checks, %d failures\n", g_checks, g_fails);
    return g_fails == 0 ? 0 : 1;
}
