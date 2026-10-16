"""Build the site kit meshes from tools/site_kit/kit_v2.json (spec Sections 2-3).

For every piece with a mesh: build it with its family script, check that its bounds match the
spec size within 0.02 m (piece frame: x length, y depth, z up, origin at the bottom centre) and
that its box proxies enclose every vertex, export the FBX to motion-planning-sim/Assets/SiteKit/
and render a 480 x 360 Workbench preview to docs/experiments/site-v2/kit-previews/. Finally
tile all previews into kit-previews/_all.png. Exits non-zero on the wrong Blender version or a
failed check (a failing piece is not exported).

Run (Blender 5.2 LTS):
  Blender --background --factory-startup --python tools/blender/site_kit/build_kit.py -- [--only ID ...]
"""
import argparse
import importlib
import json
import sys
from pathlib import Path

import bpy

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
sys.path.insert(0, str(HERE))

import common  # noqa: E402

FAMILY_MODULES = ("building", "scaffold", "fence", "cabin", "stacks", "barrier", "pipe_rack", "props",
                  "disturbances")
SITE_KIT = REPO / "motion-planning-sim/Assets/SiteKit"
PREVIEWS = REPO / "docs/experiments/site-v2/kit-previews"


def builders():
    table = {}
    for name in FAMILY_MODULES:
        if not (HERE / f"{name}.py").exists():
            continue  # a family not written yet; the missing-builder check below names its pieces
        module = importlib.import_module(name)
        overlap = table.keys() & module.BUILDERS.keys()
        if overlap:
            raise RuntimeError(f"{name} redefines {sorted(overlap)}")
        table.update(module.BUILDERS)
    return table


def parse_args():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    parser = argparse.ArgumentParser(prog="build_kit.py", description=__doc__.splitlines()[0])
    parser.add_argument("--only", nargs="+", metavar="ID", help="build only these piece ids")
    return parser.parse_args(argv)


def build_piece(piece, kit, build):
    """Build, check, export and preview one piece. Returns (row, problems)."""
    common.reset_scene()
    mesh = build(piece, kit)
    lo, hi = mesh.bounds()
    problems = []
    problem = common.bounds_problem(piece, lo, hi)
    if problem:
        problems.append(problem)
    outside = common.vertices_outside_proxies(piece, mesh.verts)
    if outside:
        sample = tuple(round(c, 4) for c in outside[0])
        problems.append(f"{piece['id']}: {len(outside)} vertices outside the proxies, e.g. {sample}")
    row = {"id": piece["id"], "triangles": mesh.triangles(),
           "measured": [round(hi[a] - lo[a], 4) for a in range(3)]}
    if problems:
        return row, problems
    obj = common.to_object(piece["id"], mesh)
    fbx = SITE_KIT / piece["mesh"]
    common.export_fbx(obj, fbx)
    row["fbx_bytes"] = fbx.stat().st_size
    common.render_preview(obj, piece, PREVIEWS / f"{piece['id']}.png")
    return row, problems


def main():
    if tuple(bpy.app.version[:2]) != common.BLENDER_VERSION:
        print(f"build_kit: Blender {bpy.app.version_string} found, the kit is pinned to "
              f"{common.BLENDER_VERSION[0]}.{common.BLENDER_VERSION[1]} LTS", file=sys.stderr)
        sys.exit(2)
    args = parse_args()
    kit = json.loads(common.kit_path(REPO).read_text())
    meshed = [p for p in kit["pieces"] if p["mesh"]]
    table = builders()
    unknown = set(args.only or ()) - {p["id"] for p in meshed}
    if unknown:
        print(f"build_kit: not mesh pieces of the kit spec: {sorted(unknown)}", file=sys.stderr)
        sys.exit(2)
    missing = [p["id"] for p in meshed if p["id"] not in table]
    if missing and not args.only:
        print(f"build_kit: no builder for {missing}", file=sys.stderr)
        sys.exit(2)
    selected = [p for p in meshed if not args.only or p["id"] in args.only]

    rows, problems = [], []
    for piece in selected:
        row, piece_problems = build_piece(piece, kit, table[piece["id"]])
        rows.append(row)
        problems += piece_problems

    previews = [PREVIEWS / f"{p['id']}.png" for p in meshed if (PREVIEWS / f"{p['id']}.png").exists()]
    if previews:
        common.contact_sheet(previews, PREVIEWS / "_all.png")

    print(f"{'piece':26s} {'measured size (m)':>26s} {'tris':>6s} {'fbx bytes':>10s}")
    for row in rows:
        size = " x ".join(f"{s:.3f}" for s in row["measured"])
        print(f"{row['id']:26s} {size:>26s} {row['triangles']:6d} {row.get('fbx_bytes', '-'):>10}")
    if problems:
        print("build_kit: FAILED\n  " + "\n  ".join(problems), file=sys.stderr)
        sys.exit(1)
    print(f"build_kit: {len(rows)} pieces built")


if __name__ == "__main__":
    try:
        main()
    except SystemExit:
        raise
    except Exception:  # noqa: BLE001 - any failure must give Blender a non-zero exit code
        import traceback

        traceback.print_exc()
        sys.exit(1)
