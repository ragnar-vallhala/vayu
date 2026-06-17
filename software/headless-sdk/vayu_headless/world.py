"""World collision-mesh build + push.

vsim_d collides the body against a BVH it mmaps (VSIM_CTL_SET_WORLD_MESH). The
GCS builds that BVH from the loaded world mesh; headless runs must do the same
or the craft flies through everything. We shell out to the `vsim_worldmesh` CLI
(reuses the GCS's own vsim::loadMesh + buildWorldBvh) so collision is identical
to what Navigator renders. Carved verbatim from sitl_lab._build_world_mesh.
"""
import os
import subprocess

from ._repo import repo_root


def _resolve_tool():
    env = os.environ.get("VSIM_WORLDMESH_BIN")
    if env:
        return env
    root = repo_root()
    # Canonical home is the SDK (PLAN.md decision #2); fall back to the legacy
    # tools/ location until the hard-cut so a move can't break a running setup.
    candidates = [
        os.path.join(root, "software", "headless-sdk", "cpp", "worldmesh",
                     "build", "vsim_worldmesh"),
        os.path.join(root, "tools", "sim_host", "worldmesh", "build",
                     "vsim_worldmesh"),
    ]
    for c in candidates:
        if os.path.exists(c):
            return c
    return candidates[0]


def build_world_mesh(w, out_path):
    """Build the world collision BVH from the GCS's selected world mesh.
    Returns (out_path, nverts, ntris, nodes, restitution, double_sided) or None."""
    mesh = w.get("worldMeshPath", "")
    if not mesh or not os.path.exists(mesh):
        return None
    tool = _resolve_tool()
    if not os.path.exists(tool):
        print(f"  [world-mesh] builder not built ({tool}); obstacles will NOT "
              f"be solid. Build it: cmake -B build -S software/headless-sdk/cpp/worldmesh")
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
