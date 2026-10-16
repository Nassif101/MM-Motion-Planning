"""Shared geometry, checks, FBX export and previews for the site kit (spec Sections 2-3).

Pieces are built in the piece frame: x along the length, y depth, z up, origin at the bottom
centre of the mesh bounds, metres. Blender's world frame is the same right-handed z-up frame,
so the bounds measured here are the piece-frame bounds the kit spec declares.
"""
import math
from pathlib import Path

import bpy
from mathutils import Vector

BLENDER_VERSION = (5, 2)
BOUNDS_TOLERANCE_M = 0.02
PROXY_TOLERANCE_M = 0.0005

# Workbench preview colours, one per shared material (linear RGB).
PREVIEW_COLOURS = {
    "asphalt": (0.04, 0.04, 0.045),
    "compacted_ground": (0.30, 0.19, 0.10),
    "gravel": (0.42, 0.40, 0.36),
    "concrete": (0.62, 0.62, 0.60),
    "blockwork": (0.45, 0.45, 0.50),
    "timber": (0.62, 0.40, 0.18),
    "galvanized_steel": (0.55, 0.62, 0.70),
    "painted_steel": (0.85, 0.55, 0.02),
    "plastic": (0.05, 0.35, 0.45),
    "cabin_paint": (0.10, 0.25, 0.55),
    "lane_marking": (0.92, 0.92, 0.92),
    "walkway_marking": (0.55, 0.85, 0.10),
}

# Box corner order: index i + 2j + 4k is the corner at (x_i, y_j, z_k).
_HEXA_FACES = ((0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5))


def _newell(points):
    n = Vector((0.0, 0.0, 0.0))
    for a, b in zip(points, points[1:] + points[:1]):
        n.x += (a.y - b.y) * (a.z + b.z)
        n.y += (a.z - b.z) * (a.x + b.x)
        n.z += (a.x - b.x) * (a.y + b.y)
    return n


class Mesh:
    """Faces of one kit piece, each with a material from the shared set."""

    def __init__(self):
        self.verts, self.faces, self.materials, self.smooth = [], [], [], []

    def add(self, verts, faces, material, smooth=False, inside=None):
        """Add a part. Faces are turned to face away from `inside` (the part's centroid by
        default, right for convex parts)."""
        if material not in PREVIEW_COLOURS:
            raise ValueError(f"material {material!r} is not in the shared set")
        verts = [Vector(v) for v in verts]
        inside = Vector(inside) if inside is not None else sum(verts, Vector()) / len(verts)
        base = len(self.verts)
        self.verts.extend(verts)
        for face in faces:
            points = [verts[i] for i in face]
            centroid = sum(points, Vector()) / len(points)
            if _newell(points).dot(centroid - inside) < 0:
                face = tuple(reversed(face))
            self.faces.append(tuple(base + i for i in face))
            self.materials.append(material)
            self.smooth.append(smooth)

    def hexa(self, corners, material):
        """Convex hexahedron from 8 corners in box order (see _HEXA_FACES)."""
        self.add(corners, _HEXA_FACES, material)

    def box(self, lo, hi, material):
        (x0, y0, z0), (x1, y1, z1) = lo, hi
        self.hexa([(x, y, z) for z in (z0, z1) for y in (y0, y1) for x in (x0, x1)], material)

    def box_c(self, centre, size, material):
        lo = [c - s / 2 for c, s in zip(centre, size)]
        hi = [c + s / 2 for c, s in zip(centre, size)]
        self.box(lo, hi, material)

    def chamfer_box(self, lo, hi, chamfer, material):
        """Box with every edge chamfered by `chamfer` (24 vertices, 26 faces)."""
        c = Vector(lo).lerp(Vector(hi), 0.5)
        h = (Vector(hi) - Vector(lo)) / 2
        verts, index = [], {}
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    for axis in range(3):
                        inset = [h[a] - chamfer if a != axis else h[a] for a in range(3)]
                        verts.append((c.x + sx * inset[0], c.y + sy * inset[1], c.z + sz * inset[2]))
                        index[(sx, sy, sz, axis)] = len(verts) - 1
        faces = []
        for axis in range(3):
            for s in (-1, 1):
                ring = []
                for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
                    signs = [0, 0, 0]
                    signs[axis] = s
                    others = [x for x in range(3) if x != axis]
                    signs[others[0]], signs[others[1]] = a, b
                    ring.append(index[(*signs, axis)])
                faces.append(tuple(ring))
        for a1 in range(3):
            for a2 in range(a1 + 1, 3):
                free = 3 - a1 - a2
                for s1 in (-1, 1):
                    for s2 in (-1, 1):
                        quad = []
                        for sf, axis in ((-1, a1), (-1, a2), (1, a2), (1, a1)):
                            signs = [0, 0, 0]
                            signs[a1], signs[a2], signs[free] = s1, s2, sf
                            quad.append(index[(*signs, axis)])
                        faces.append(tuple(quad))
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    faces.append(tuple(index[(sx, sy, sz, axis)] for axis in range(3)))
        self.add(verts, faces, material)

    def cylinder(self, p0, p1, radius, material, sides=12, caps=True):
        """Smooth-sided cylinder from p0 to p1; caps are separate flat faces."""
        p0, p1 = Vector(p0), Vector(p1)
        axis = (p1 - p0).normalized()
        u = axis.cross(Vector((0, 0, 1)) if abs(axis.z) < 0.9 else Vector((1, 0, 0))).normalized()
        v = axis.cross(u)
        ring = [u * math.cos(2 * math.pi * k / sides) + v * math.sin(2 * math.pi * k / sides)
                for k in range(sides)]
        side = [p + r * radius for p in (p0, p1) for r in ring]
        faces = [(k, (k + 1) % sides, sides + (k + 1) % sides, sides + k) for k in range(sides)]
        self.add(side, faces, material, smooth=True, inside=(p0 + p1) / 2)
        if caps:
            for p in (p0, p1):
                self.add([p + r * radius for r in ring], [tuple(range(sides))], material,
                         inside=(p0 + p1) / 2)

    def prism(self, profile, axis, start, end, material):
        """Convex 2D profile extruded along `axis` ('x', 'y' or 'z') from start to end.

        The profile gives the two other coordinates in order (x: (y, z); y: (x, z); z: (x, y)).
        """
        a = "xyz".index(axis)
        others = [i for i in range(3) if i != a]

        def point(p, t):
            co = [0.0, 0.0, 0.0]
            co[a], co[others[0]], co[others[1]] = t, p[0], p[1]
            return co

        n = len(profile)
        verts = [point(p, start) for p in profile] + [point(p, end) for p in profile]
        faces = [tuple(range(n)), tuple(range(n, 2 * n))]
        faces += [(k, (k + 1) % n, n + (k + 1) % n, n + k) for k in range(n)]
        self.add(verts, faces, material)

    def triangles(self):
        return sum(len(f) - 2 for f in self.faces)

    def bounds(self):
        lo = tuple(min(v[a] for v in self.verts) for a in range(3))
        hi = tuple(max(v[a] for v in self.verts) for a in range(3))
        return lo, hi


def reset_scene():
    for obj in list(bpy.data.objects):
        bpy.data.objects.remove(obj, do_unlink=True)
    for collection in (bpy.data.meshes, bpy.data.materials, bpy.data.cameras, bpy.data.lights,
                       bpy.data.curves, bpy.data.images):
        for block in list(collection):
            collection.remove(block)


def material(name):
    mat = bpy.data.materials.get(name)
    if mat is None:
        mat = bpy.data.materials.new(name)
        colour = PREVIEW_COLOURS.get(name, (0.0, 0.0, 0.0))
        mat.diffuse_color = (*colour, 1.0)
        mat.roughness = 0.6
    return mat


def to_object(name, mesh):
    """One mesh object with a slot per material (named from the shared set) and box UVs."""
    data = bpy.data.meshes.new(name)
    data.from_pydata([tuple(v) for v in mesh.verts], [], mesh.faces)
    slots = list(dict.fromkeys(mesh.materials))
    for slot in slots:
        data.materials.append(material(slot))
    for poly, mat, smooth in zip(data.polygons, mesh.materials, mesh.smooth):
        poly.material_index = slots.index(mat)
        poly.use_smooth = smooth
    box_project_uvs(data)
    data.validate()
    obj = bpy.data.objects.new(name, data)
    bpy.context.scene.collection.objects.link(obj)
    return obj


def box_project_uvs(data):
    """Cube projection at 1 m per texture tile: each face maps the two axes across its normal."""
    layer = data.uv_layers.new(name="UVMap")
    for poly in data.polygons:
        axis = max(range(3), key=lambda a: abs(poly.normal[a]))
        u_axis, v_axis = [a for a in range(3) if a != axis]
        for loop_index in poly.loop_indices:
            co = data.vertices[data.loops[loop_index].vertex_index].co
            layer.data[loop_index].uv = (co[u_axis], co[v_axis])


def bounds_problem(piece, lo, hi, tolerance=BOUNDS_TOLERANCE_M):
    """None when the mesh bounds match the spec size (origin at the bottom centre), else why."""
    sx, sy, sz = piece["size"]
    want_lo, want_hi = (-sx / 2, -sy / 2, 0.0), (sx / 2, sy / 2, sz)
    worst = max(max(abs(lo[a] - want_lo[a]), abs(hi[a] - want_hi[a])) for a in range(3))
    if worst > tolerance:
        got = " x ".join(f"{hi[a] - lo[a]:.3f}" for a in range(3))
        return (f"{piece['id']}: mesh bounds {tuple(round(x, 3) for x in lo)}..{tuple(round(x, 3) for x in hi)}"
                f" ({got}) vs spec size {piece['size']} (off by {worst:.3f} m)")
    return None


def _inside_proxy(point, proxy, tolerance):
    (cx, cy, cz), (sx, sy, sz), yaw = proxy["centre"], proxy["size"], proxy["yaw"]
    dx, dy = point[0] - cx, point[1] - cy
    c, s = math.cos(-yaw), math.sin(-yaw)
    lx, ly = c * dx - s * dy, s * dx + c * dy
    return (abs(lx) <= sx / 2 + tolerance and abs(ly) <= sy / 2 + tolerance
            and abs(point[2] - cz) <= sz / 2 + tolerance)


def vertices_outside_proxies(piece, verts, tolerance=PROXY_TOLERANCE_M):
    """Vertices not inside any of the piece's box proxies (proxies must enclose the mesh)."""
    proxies = piece["proxies"]
    return [v for v in verts if not any(_inside_proxy(v, p, tolerance) for p in proxies)]


def export_fbx(obj, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    for other in bpy.context.scene.objects:
        other.select_set(False)
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.export_scene.fbx(
        filepath=str(path), use_selection=True, object_types={"MESH"},
        axis_forward="-Z", axis_up="Y", apply_scale_options="FBX_SCALE_UNITS",
        mesh_smooth_type="FACE", use_mesh_modifiers=True, add_leaf_bones=False,
        bake_anim=False, use_custom_props=False, path_mode="AUTO", embed_textures=False)


def _scale_bar(lo, hi):
    """A 1 m bar of ten alternating 0.1 m segments in front of the piece."""
    span = max(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2])
    t = min(max(0.012 * span, 0.008), 0.05)
    y = lo[1] - max(0.15, 0.08 * span)
    bar = Mesh()
    for k in range(10):
        x0 = lo[0] + 0.1 * k
        bar.box((x0, y - t / 2, 0.0), (x0 + 0.1, y + t / 2, t), "asphalt" if k % 2 == 0 else "lane_marking")
    return to_object("scale_bar_1m", bar)


def _label(camera, text, half_height, aspect):
    curve = bpy.data.curves.new("label", type="FONT")
    curve.body = text
    curve.size = 0.075 * half_height
    curve.align_x, curve.align_y = "LEFT", "TOP"
    label = bpy.data.objects.new("label", curve)
    bpy.context.scene.collection.objects.link(label)
    ink = bpy.data.materials.new("label_ink")
    ink.diffuse_color = (0.0, 0.0, 0.0, 1.0)
    curve.materials.append(ink)
    label.parent = camera
    label.location = (-0.95 * half_height * aspect, 0.93 * half_height, -1.0)
    return label


def render_preview(obj, piece, path, width=480, height=360):
    """3/4 Workbench view framing the piece with a 1 m scale bar and a label."""
    scene = bpy.context.scene
    lo, hi = obj_bounds(obj)
    bar = _scale_bar(lo, hi)
    ground = Mesh()
    pad = 0.6 * max(hi[0] - lo[0], hi[1] - lo[1], 1.0)
    ground.box((lo[0] - pad, lo[1] - pad, -0.002), (hi[0] + pad, hi[1] + pad, -0.001), "lane_marking")
    ground_obj = to_object("ground", ground)
    ground_obj.data.materials[0] = bpy.data.materials.new("ground_preview")
    ground_obj.data.materials[0].diffuse_color = (0.80, 0.80, 0.78, 1.0)

    points = [Vector(v) for v in (lo, hi, obj_bounds(bar)[0], obj_bounds(bar)[1])]
    box_lo = Vector([min(p[a] for p in points) for a in range(3)])
    box_hi = Vector([max(p[a] for p in points) for a in range(3)])
    centre = (box_lo + box_hi) / 2

    cam_data = bpy.data.cameras.new("preview")
    cam_data.lens_unit = "FOV"
    cam_data.sensor_fit = "VERTICAL"
    cam_data.angle = math.radians(30.0)
    cam_data.clip_start, cam_data.clip_end = 0.01, 500.0
    camera = bpy.data.objects.new("preview", cam_data)
    scene.collection.objects.link(camera)
    direction = Vector((0.9, -1.35, 0.85)).normalized()
    rotation = (-direction).to_track_quat("-Z", "Y")
    right, up = rotation @ Vector((1, 0, 0)), rotation @ Vector((0, 1, 0))
    ty = math.tan(cam_data.angle / 2)
    tx = ty * width / height
    distance = 0.0
    for corner in (Vector((x, y, z)) for x in (box_lo.x, box_hi.x) for y in (box_lo.y, box_hi.y)
                   for z in (box_lo.z, box_hi.z)):
        q = corner - centre
        x, y = q.dot(right), q.dot(up)
        # Keep every corner inside the frame, with the top 25 % left for the label.
        limit_y = 0.72 * ty if y > 0 else 0.92 * ty
        distance = max(distance, q.dot(direction) + abs(x) / (0.92 * tx), q.dot(direction) + abs(y) / limit_y)
    camera.location = centre + direction * distance
    camera.rotation_euler = rotation.to_euler()
    scene.camera = camera

    sx, sy, sz = piece["size"]
    _label(camera, f"{piece['id']}\n{sx:g} x {sy:g} x {sz:g} m  (bar 1 m)",
           math.tan(cam_data.angle / 2), width / height)

    scene.render.engine = "BLENDER_WORKBENCH"
    scene.render.resolution_x, scene.render.resolution_y = width, height
    scene.render.resolution_percentage = 100
    scene.render.film_transparent = False
    scene.render.image_settings.file_format = "PNG"
    scene.render.image_settings.color_mode = "RGB"
    scene.render.image_settings.compression = 100
    scene.view_settings.view_transform = "Standard"
    shading = scene.display.shading
    shading.light = "STUDIO"
    shading.color_type = "MATERIAL"
    shading.show_shadows = True
    shading.show_cavity = True
    shading.cavity_type = "WORLD"
    shading.show_object_outline = True
    scene.world.color = (0.93, 0.94, 0.96)
    scene.display.render_aa = "8"
    path.parent.mkdir(parents=True, exist_ok=True)
    scene.render.filepath = str(path)
    bpy.ops.render.render(write_still=True)


def obj_bounds(obj):
    coords = [obj.matrix_world @ v.co for v in obj.data.vertices]
    lo = tuple(min(c[a] for c in coords) for a in range(3))
    hi = tuple(max(c[a] for c in coords) for a in range(3))
    return lo, hi


def contact_sheet(paths, out, columns=6, tile=(480, 360)):
    """Grid of the preview PNGs (each carries its own label), row by row from the top."""
    import numpy as np

    w, h = tile
    rows = math.ceil(len(paths) / columns)
    sheet = np.ones((rows * h, columns * w, 4), dtype=np.float32)
    for n, path in enumerate(paths):
        image = bpy.data.images.load(str(path))
        if tuple(image.size) != tile:
            raise ValueError(f"{path} is {tuple(image.size)}, expected {tile}")
        pixels = np.empty(w * h * 4, dtype=np.float32)
        image.pixels.foreach_get(pixels)
        row, col = divmod(n, columns)
        top = (rows - 1 - row) * h  # Blender images are stored bottom row first
        sheet[top:top + h, col * w:(col + 1) * w] = pixels.reshape(h, w, 4)
        bpy.data.images.remove(image)
    result = bpy.data.images.new("contact_sheet", columns * w, rows * h, alpha=False)
    result.pixels.foreach_set(sheet.ravel())
    result.filepath_raw = str(out)
    result.file_format = "PNG"
    result.save()
    bpy.data.images.remove(result)


def kit_path(repo):
    return Path(repo) / "tools/site_kit/kit_v2.json"
