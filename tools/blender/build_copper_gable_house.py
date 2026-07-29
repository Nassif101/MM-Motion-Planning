"""Deterministic Blender build recipe for the Copper Gable House."""

import math
import os

import bpy
from mathutils import Vector


COLLECTION_NAME = "Codex_CopperGableHouse"
PREFIX = "CH_"
STAGE = globals().get("STAGE", "all")
OUTPUT_PATH = globals().get(
    "OUTPUT_PATH",
    r"C:\Users\Heisenberg\Documents\Motion Planner MA\mt-mohamad-nassif\artifacts\copper_gable_house.png",
)


def collection():
    coll = bpy.data.collections.get(COLLECTION_NAME)
    if coll is None:
        coll = bpy.data.collections.new(COLLECTION_NAME)
        bpy.context.scene.collection.children.link(coll)
    return coll


def clear_collection():
    coll = bpy.data.collections.get(COLLECTION_NAME)
    if coll is None:
        return
    for obj in list(coll.objects):
        bpy.data.objects.remove(obj, do_unlink=True)


def relink(obj):
    target = collection()
    for source in list(obj.users_collection):
        source.objects.unlink(obj)
    target.objects.link(obj)
    root = bpy.data.objects.get(PREFIX + "HouseRoot")
    if root is not None and obj is not root and obj.type not in {"CAMERA", "LIGHT"}:
        obj.parent = root
    return obj


def material(name, color, metallic=0.0, roughness=0.5):
    mat = bpy.data.materials.get(name)
    if mat is None:
        mat = bpy.data.materials.new(name)
    mat.diffuse_color = (*color, 1.0)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get("Principled BSDF")
    bsdf.inputs["Base Color"].default_value = (*color, 1.0)
    bsdf.inputs["Metallic"].default_value = metallic
    bsdf.inputs["Roughness"].default_value = roughness
    return mat


def cube(name, location, dimensions, mat=None, rotation=(0.0, 0.0, 0.0)):
    bpy.ops.mesh.primitive_cube_add(location=location)
    obj = bpy.context.active_object
    obj.name = name
    obj.dimensions = dimensions
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    obj.rotation_euler = rotation
    if mat is not None:
        obj.data.materials.append(mat)
    return relink(obj)


def prism_gable(name, x_center, thickness, wall_mat):
    y0, y1 = -3.2, 3.2
    z0, z1 = 4.85, 7.05
    xa, xb = x_center - thickness / 2.0, x_center + thickness / 2.0
    vertices = [
        (xa, y0, z0), (xa, y1, z0), (xa, 0.0, z1),
        (xb, y0, z0), (xb, y1, z0), (xb, 0.0, z1),
    ]
    faces = [
        (0, 2, 1), (3, 4, 5),
        (0, 3, 5, 2), (1, 2, 5, 4), (0, 1, 4, 3),
    ]
    mesh = bpy.data.meshes.new(name + "_Mesh")
    mesh.from_pydata(vertices, [], faces)
    mesh.update()
    obj = bpy.data.objects.new(name, mesh)
    collection().objects.link(obj)
    obj.data.materials.append(wall_mat)
    root = bpy.data.objects.get(PREFIX + "HouseRoot")
    if root is not None:
        obj.parent = root
    return obj


def boolean_recess(target, name, location, dimensions):
    cutter = cube(name, location, dimensions)
    modifier = target.modifiers.new(name + "_Boolean", "BOOLEAN")
    modifier.operation = "DIFFERENCE"
    modifier.solver = "EXACT"
    modifier.object = cutter
    bpy.context.view_layer.objects.active = target
    target.select_set(True)
    bpy.ops.object.modifier_apply(modifier=modifier.name)
    target.select_set(False)
    bpy.data.objects.remove(cutter, do_unlink=True)


def frame_front(name, center_x, center_z, width, height, frame_width, depth, mat):
    y = -3.235
    parts = [
        (name + "_Left", (center_x - width / 2.0, y, center_z), (frame_width, depth, height)),
        (name + "_Right", (center_x + width / 2.0, y, center_z), (frame_width, depth, height)),
        (name + "_Top", (center_x, y, center_z + height / 2.0), (width, depth, frame_width)),
        (name + "_Bottom", (center_x, y, center_z - height / 2.0), (width, depth, frame_width)),
    ]
    return [cube(part_name, loc, dims, mat) for part_name, loc, dims in parts]


def frame_side(name, center_y, center_z, width, height, frame_width, depth, mat):
    x = 4.235
    parts = [
        (name + "_Front", (x, center_y - width / 2.0, center_z), (depth, frame_width, height)),
        (name + "_Rear", (x, center_y + width / 2.0, center_z), (depth, frame_width, height)),
        (name + "_Top", (x, center_y, center_z + height / 2.0), (depth, width, frame_width)),
        (name + "_Bottom", (x, center_y, center_z - height / 2.0), (depth, width, frame_width)),
    ]
    return [cube(part_name, loc, dims, mat) for part_name, loc, dims in parts]


def ico(name, location, scale, mat):
    bpy.ops.mesh.primitive_ico_sphere_add(subdivisions=1, radius=1.0, location=location)
    obj = bpy.context.active_object
    obj.name = name
    obj.scale = scale
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    obj.data.materials.append(mat)
    return relink(obj)


def look_at(obj, target):
    direction = Vector(target) - obj.location
    obj.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()


def build_blockout():
    clear_collection()
    coll = collection()
    root = bpy.data.objects.new(PREFIX + "HouseRoot", None)
    root.empty_display_type = "CUBE"
    root.empty_display_size = 0.6
    coll.objects.link(root)

    grass = material(PREFIX + "Grass", (0.17, 0.29, 0.13), roughness=0.9)
    stone = material(PREFIX + "Foundation", (0.28, 0.30, 0.29), roughness=0.8)
    stucco = material(PREFIX + "WarmStucco", (0.72, 0.55, 0.36), roughness=0.72)

    cube(PREFIX + "GardenPad", (0.0, 0.0, 0.15), (14.0, 12.0, 0.3), grass)
    cube(PREFIX + "Foundation", (0.0, 0.0, 0.50), (9.0, 7.0, 0.4), stone)
    cube(PREFIX + "MainBody", (0.0, 0.0, 2.75), (8.4, 6.4, 4.2), stucco)
    result = {
        "stage": "blockout",
        "objects": [obj.name for obj in coll.objects],
        "body_bounds": {"x": [-4.2, 4.2], "y": [-3.2, 3.2], "z": [0.65, 4.85]},
    }
    return result


def build_architecture():
    body = bpy.data.objects[PREFIX + "MainBody"]
    stucco = bpy.data.materials[PREFIX + "WarmStucco"]
    roof_mat = material(PREFIX + "CharcoalRoof", (0.055, 0.065, 0.075), metallic=0.08, roughness=0.32)
    bronze = material(PREFIX + "DarkBronze", (0.07, 0.045, 0.025), metallic=0.72, roughness=0.26)
    glass = material(PREFIX + "WindowGlass", (0.05, 0.22, 0.28), metallic=0.18, roughness=0.12)
    wood = material(PREFIX + "DoorWood", (0.27, 0.10, 0.045), roughness=0.42)

    boolean_recess(body, PREFIX + "DoorPocket", (1.75, -3.18, 1.98), (1.6, 0.72, 2.72))
    boolean_recess(body, PREFIX + "WindowPocket", (-1.65, -3.18, 2.70), (3.2, 0.72, 1.65))

    cube(PREFIX + "Door", (1.75, -2.835, 1.98), (1.35, 0.08, 2.48), wood)
    frame_front(PREFIX + "DoorFrame", 1.75, 1.98, 1.52, 2.65, 0.12, 0.13, bronze)
    cube(PREFIX + "DoorHandle", (2.22, -3.305, 2.00), (0.08, 0.08, 0.32), bronze)

    cube(PREFIX + "FrontGlass", (-1.65, -2.835, 2.70), (2.95, 0.07, 1.42), glass)
    frame_front(PREFIX + "WindowFrame", -1.65, 2.70, 3.12, 1.58, 0.11, 0.13, bronze)
    cube(PREFIX + "WindowMullionV", (-1.65, -3.24, 2.70), (0.09, 0.14, 1.50), bronze)
    cube(PREFIX + "WindowMullionH", (-1.65, -3.24, 2.70), (3.0, 0.14, 0.09), bronze)

    angle = math.atan2(2.2, 3.5)
    slope = math.sqrt(3.5 ** 2 + 2.2 ** 2)
    cube(PREFIX + "RoofFront", (0.0, -1.75, 5.95), (9.2, slope, 0.26), roof_mat, (angle, 0.0, 0.0))
    cube(PREFIX + "RoofRear", (0.0, 1.75, 5.95), (9.2, slope, 0.26), roof_mat, (-angle, 0.0, 0.0))
    prism_gable(PREFIX + "GableLeft", -4.19, 0.18, stucco)
    prism_gable(PREFIX + "GableRight", 4.19, 0.18, stucco)

    result = {
        "stage": "architecture",
        "openings": ["front door recess", "front window recess"],
        "roof": {"rise": 2.2, "overhang": 0.4, "thickness": 0.26},
    }
    return result


def build_details():
    bronze = bpy.data.materials[PREFIX + "DarkBronze"]
    roof_mat = bpy.data.materials[PREFIX + "CharcoalRoof"]
    copper = material(PREFIX + "WeatheredCopper", (0.24, 0.33, 0.25), metallic=0.82, roughness=0.34)
    timber = material(PREFIX + "PorchTimber", (0.20, 0.085, 0.035), roughness=0.5)
    paving = material(PREFIX + "Paving", (0.39, 0.36, 0.31), roughness=0.85)
    foliage = material(PREFIX + "Foliage", (0.09, 0.23, 0.075), roughness=0.9)
    chimney = material(PREFIX + "ChimneyBrick", (0.33, 0.12, 0.075), roughness=0.88)

    cube(PREFIX + "PorchSlab", (1.75, -3.86, 0.73), (3.1, 1.45, 0.18), paving)
    cube(PREFIX + "PorchColumnLeft", (0.72, -4.25, 2.08), (0.18, 0.18, 2.70), timber)
    cube(PREFIX + "PorchColumnRight", (2.78, -4.25, 2.08), (0.18, 0.18, 2.70), timber)
    cube(PREFIX + "CopperCanopy", (1.75, -3.84, 3.48), (2.65, 1.75, 0.16), copper, (math.radians(-7.0), 0.0, 0.0))
    cube(PREFIX + "StepUpper", (1.75, -4.68, 0.58), (3.0, 0.55, 0.18), paving)
    cube(PREFIX + "StepLower", (1.75, -5.03, 0.43), (2.7, 0.42, 0.14), paving)

    cube(PREFIX + "Chimney", (2.75, 1.35, 6.55), (0.82, 0.82, 2.25), chimney)
    cube(PREFIX + "ChimneyCap", (2.75, 1.35, 7.72), (1.02, 1.02, 0.14), roof_mat)

    for index, y in enumerate((-5.35, -4.70, -4.05, -3.40)):
        cube(PREFIX + f"PathStone_{index + 1:02d}", (1.75, y, 0.355), (1.4, 0.46, 0.09), paving)

    shrub_specs = [
        (-3.55, -3.85, 0.78, (0.72, 0.62, 0.58)),
        (-2.65, -4.05, 0.68, (0.58, 0.52, 0.48)),
        (3.55, -3.75, 0.72, (0.62, 0.56, 0.52)),
    ]
    for index, (x, y, z, scale) in enumerate(shrub_specs):
        ico(PREFIX + f"Shrub_{index + 1:02d}", (x, y, z), scale, foliage)

    cube(PREFIX + "EaveFasciaFront", (0.0, -3.54, 4.82), (9.22, 0.12, 0.28), bronze)
    cube(PREFIX + "EaveFasciaRear", (0.0, 3.54, 4.82), (9.22, 0.12, 0.28), bronze)

    result = {
        "stage": "details",
        "contact_audit": {
            "porch_columns_to_slab": "column bottoms z=0.73, slab top z=0.82; intentional 0.09 m overlap",
            "canopy_to_columns": "column tops z=3.43, canopy spans approximately z=3.24..3.72",
            "chimney_to_roof": "chimney penetrates rear roof intentionally",
        },
    }
    return result


def build_refinement():
    body = bpy.data.objects[PREFIX + "MainBody"]
    bronze = bpy.data.materials[PREFIX + "DarkBronze"]
    glass = bpy.data.materials[PREFIX + "WindowGlass"]

    boolean_recess(body, PREFIX + "SideWindowPocket", (4.18, 0.45, 2.72), (0.72, 2.5, 1.55))
    cube(PREFIX + "SideGlass", (3.835, 0.45, 2.72), (0.07, 2.27, 1.32), glass)
    frame_side(PREFIX + "SideWindowFrame", 0.45, 2.72, 2.42, 1.48, 0.11, 0.13, bronze)
    cube(PREFIX + "SideWindowMullion", (4.24, 0.45, 2.72), (0.14, 0.09, 1.38), bronze)
    result = {
        "stage": "refinement",
        "change": "Added a recessed right-side window to balance the visible elevation.",
    }
    return result


def final_cleanup():
    default_cube = bpy.data.objects.get("Cube")
    if default_cube is not None and not default_cube.name.startswith(PREFIX):
        default_cube.hide_render = True
        default_cube.hide_set(True)
    bpy.ops.object.select_all(action="DESELECT")
    result = {
        "stage": "final_cleanup",
        "default_cube_preserved_but_hidden": default_cube is not None,
    }
    return result


def setup_presentation():
    scene = bpy.context.scene
    for object_name in (
        PREFIX + "PresentationCamera",
        PREFIX + "Sun",
        PREFIX + "Fill",
        PREFIX + "Rim",
    ):
        old = bpy.data.objects.get(object_name)
        if old is not None:
            bpy.data.objects.remove(old, do_unlink=True)

    engine_items = {item.identifier for item in scene.render.bl_rna.properties["engine"].enum_items}
    scene.render.engine = "BLENDER_EEVEE_NEXT" if "BLENDER_EEVEE_NEXT" in engine_items else "BLENDER_EEVEE"
    scene.render.resolution_x = 900
    scene.render.resolution_y = 700
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.film_transparent = False
    scene.world.color = (0.035, 0.045, 0.065)
    if scene.world.use_nodes:
        background = scene.world.node_tree.nodes.get("Background")
        background.inputs["Color"].default_value = (0.028, 0.045, 0.072, 1.0)
        background.inputs["Strength"].default_value = 0.45

    camera_data = bpy.data.cameras.new(PREFIX + "PresentationCamera_Data")
    camera = bpy.data.objects.new(PREFIX + "PresentationCamera", camera_data)
    collection().objects.link(camera)
    camera.location = (13.5, -16.5, 10.2)
    camera.data.lens = 52
    look_at(camera, (0.0, 0.0, 3.0))
    scene.camera = camera

    sun_data = bpy.data.lights.new(PREFIX + "Sun_Data", "SUN")
    sun_data.energy = 2.2
    sun_data.angle = math.radians(18.0)
    sun = bpy.data.objects.new(PREFIX + "Sun", sun_data)
    collection().objects.link(sun)
    sun.rotation_euler = (math.radians(28.0), math.radians(-20.0), math.radians(-35.0))

    area_data = bpy.data.lights.new(PREFIX + "Fill_Data", "AREA")
    area_data.energy = 900.0
    area_data.shape = "DISK"
    area_data.size = 7.0
    area = bpy.data.objects.new(PREFIX + "Fill", area_data)
    collection().objects.link(area)
    area.location = (-7.0, -8.0, 9.0)
    look_at(area, (0.0, 0.0, 2.5))

    rim_data = bpy.data.lights.new(PREFIX + "Rim_Data", "AREA")
    rim_data.energy = 700.0
    rim_data.shape = "DISK"
    rim_data.size = 5.0
    rim = bpy.data.objects.new(PREFIX + "Rim", rim_data)
    collection().objects.link(rim)
    rim.location = (9.0, 4.0, 7.5)
    look_at(rim, (2.5, 0.5, 3.0))

    os.makedirs(os.path.dirname(OUTPUT_PATH), exist_ok=True)
    scene.render.filepath = OUTPUT_PATH
    bpy.ops.render.render(write_still=True)
    result = {"stage": "presentation", "render_path": OUTPUT_PATH}
    return result


def validate_house():
    scene = bpy.context.scene
    coll = collection()
    camera = bpy.data.objects[PREFIX + "PresentationCamera"]
    garden = bpy.data.objects[PREFIX + "GardenPad"]
    inspection_dir = os.path.join(os.path.dirname(OUTPUT_PATH), "copper_gable_inspection")
    os.makedirs(inspection_dir, exist_ok=True)

    old_camera_type = camera.data.type
    old_location = camera.location.copy()
    old_rotation = camera.rotation_euler.copy()
    old_resolution = (scene.render.resolution_x, scene.render.resolution_y, scene.render.resolution_percentage)
    old_filepath = scene.render.filepath
    old_garden_hidden = garden.hide_render

    camera.data.type = "ORTHO"
    camera.data.ortho_scale = 15.0
    scene.render.resolution_x = 420
    scene.render.resolution_y = 420
    scene.render.resolution_percentage = 100
    views = {
        "front": ((0.0, -18.0, 3.2), (0.0, 0.0, 3.0)),
        "rear": ((0.0, 18.0, 3.2), (0.0, 0.0, 3.0)),
        "left": ((-18.0, 0.0, 3.2), (0.0, 0.0, 3.0)),
        "right": ((18.0, 0.0, 3.2), (0.0, 0.0, 3.0)),
        "top": ((0.0, 0.0, 20.0), (0.0, 0.0, 2.5)),
        "underside": ((0.0, -14.0, -9.0), (0.0, -0.5, 1.2)),
    }
    rendered_views = []
    for view_name, (location, target) in views.items():
        garden.hide_render = view_name == "underside"
        camera.location = location
        look_at(camera, target)
        path = os.path.join(inspection_dir, view_name + ".png")
        scene.render.filepath = path
        bpy.ops.render.render(write_still=True)
        rendered_views.append(path)

    camera.data.type = old_camera_type
    camera.location = old_location
    camera.rotation_euler = old_rotation
    scene.render.resolution_x, scene.render.resolution_y, scene.render.resolution_percentage = old_resolution
    scene.render.filepath = old_filepath
    garden.hide_render = old_garden_hidden

    prefixed_objects = [obj for obj in bpy.data.objects if obj.name.startswith(PREFIX)]
    misplaced = [
        obj.name
        for obj in prefixed_objects
        if obj.name != PREFIX + "HouseRoot" and list(obj.users_collection) != [coll]
    ]
    shared_meshes = [
        obj.name
        for obj in prefixed_objects
        if obj.type == "MESH" and obj.data is not None and obj.data.users > 1
    ]
    leftover_cutters = [
        obj.name
        for obj in prefixed_objects
        if "Pocket" in obj.name
    ]

    porch = bpy.data.objects[PREFIX + "PorchSlab"]
    left_column = bpy.data.objects[PREFIX + "PorchColumnLeft"]
    canopy = bpy.data.objects[PREFIX + "CopperCanopy"]
    porch_top = porch.location.z + porch.dimensions.z / 2.0
    column_bottom = left_column.location.z - left_column.dimensions.z / 2.0
    column_top = left_column.location.z + left_column.dimensions.z / 2.0
    canopy_world_z = [(canopy.matrix_world @ Vector(corner)).z for corner in canopy.bound_box]
    contact = {
        "column_into_slab_m": round(porch_top - column_bottom, 4),
        "column_into_canopy_m": round(column_top - min(canopy_world_z), 4),
    }

    result = {
        "stage": "validation",
        "object_count": len(prefixed_objects),
        "collection_membership_ok": not misplaced,
        "misplaced_objects": misplaced,
        "shared_meshes": shared_meshes,
        "leftover_cutters": leftover_cutters,
        "contact_overlaps": contact,
        "six_side_renders": rendered_views,
    }
    return result


if STAGE == "blockout":
    result = build_blockout()
elif STAGE == "architecture":
    result = build_architecture()
elif STAGE == "details":
    result = build_details()
elif STAGE == "refinement":
    result = build_refinement()
elif STAGE == "final_cleanup":
    result = final_cleanup()
elif STAGE == "presentation":
    result = setup_presentation()
elif STAGE == "validation":
    result = validate_house()
elif STAGE == "all":
    build_blockout()
    build_architecture()
    build_details()
    build_refinement()
    final_cleanup()
    setup_presentation()
    result = validate_house()
else:
    raise ValueError(f"Unknown Copper Gable House stage: {STAGE}")
