"""World collision-mesh build + push.

vsim_d collides the body against a BVH it mmaps (VSIM_CTL_SET_WORLD_MESH). The
GCS builds that BVH from the loaded world mesh; headless runs must do the same
or the craft flies through everything. We shell out to the `vsim_worldmesh` CLI
(reuses the GCS's own vsim::loadMesh + buildWorldBvh) so collision is identical
to what Navigator renders. Carved verbatim from sitl_lab._build_world_mesh.
"""
import os
import subprocess

from . import paths


def build_world_mesh(w, out_path):
    """Build the world collision BVH from the GCS's selected world mesh.
    Returns (out_path, nverts, ntris, nodes, restitution, double_sided) or None."""
    mesh = w.get("worldMeshPath", "")
    if not mesh or not os.path.exists(mesh):
        return None
    tool = paths.worldmesh_bin()
    if not os.path.exists(tool):
        print(f"  [world-mesh] builder not built ({tool}); obstacles will NOT "
              f"be solid. Build it: cmake -B build -S navigator/headless-sdk/cpp/worldmesh")
        return None
    scale = w.get("worldScale", "1")
    up = "1" if str(w.get("worldUpAxis", "0")) in ("1", "Y", "y") else "0"
    ox, oy, oz = (w.get("worldMeshOffX", "0"), w.get("worldMeshOffY", "0"),
                  w.get("worldMeshOffZ", "0"))
    dbl = "1" if str(w.get("worldMeshDoubleSided", "true")).lower() in \
        ("1", "true") else "0"
    rest = float(w.get("worldMeshRestitution", "0.3"))
    try:
        out = subprocess.check_output(
            [tool, mesh, scale, up, ox, oy, oz, dbl, out_path],
            stderr=subprocess.STDOUT).decode().strip()
        nverts, ntris, nodes, _bytes = (int(x) for x in out.split())
    except (subprocess.CalledProcessError, ValueError) as e:
        print(f"  [world-mesh] build failed: {e}")
        return None
    return (out_path, nverts, ntris, nodes, rest, dbl == "1")
