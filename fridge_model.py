"""
FridgeAI 3D Model — Blender Python Script
==========================================
Run this in Blender's Scripting tab (Text > Run Script).
Tested on Blender 3.6+ and 4.x.

Scene includes:
  - Semi-transparent fridge body (walls, no front face)
  - 3 glass shelves at realistic heights
  - 3 cameras: top-down (teal) + 2 corner (blue, red)
  - FOV cones per camera in matching colours
  - Sample food items on shelves
  - HDRI-style dark world background (matching project palette)
  - Area lights + scene camera pre-aimed at the fridge
"""

import bpy
import math
from mathutils import Vector, Euler

# ──────────────────────────────────────────────
# 0. SCENE RESET
# ──────────────────────────────────────────────
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)

for col in list(bpy.data.collections):
    bpy.data.collections.remove(col)

# ──────────────────────────────────────────────
# 1. COLLECTION
# ──────────────────────────────────────────────
fridge_col = bpy.data.collections.new("FridgeAI")
bpy.context.scene.collection.children.link(fridge_col)


def link(obj):
    for c in list(obj.users_collection):
        c.objects.unlink(obj)
    fridge_col.objects.link(obj)


# ──────────────────────────────────────────────
# 2. MATERIALS
# ──────────────────────────────────────────────
def make_mat(name, color_rgb, alpha=1.0, metallic=0.0, roughness=0.5, emission=None):
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links

    bsdf = nodes["Principled BSDF"]
    bsdf.inputs["Base Color"].default_value = (*color_rgb, 1.0)
    bsdf.inputs["Metallic"].default_value = metallic
    bsdf.inputs["Roughness"].default_value = roughness
    bsdf.inputs["Alpha"].default_value = alpha

    if emission:
        bsdf.inputs["Emission Color"].default_value = (*emission, 1.0)
        bsdf.inputs["Emission Strength"].default_value = 1.5

    if alpha < 1.0:
        # blend_method / shadow_method removed in Blender 4.0; use surface_render_method if available
        if hasattr(mat, 'surface_render_method'):
            mat.surface_render_method = 'BLENDED'   # Blender 4.x EEVEE Next
        elif hasattr(mat, 'blend_method'):
            mat.blend_method = 'BLEND'               # Blender 3.x

    return mat


# Project palette
TEAL  = (0.00, 0.83, 0.67)
BLUE  = (0.23, 0.61, 1.00)
RED   = (1.00, 0.30, 0.42)
WHITE = (0.92, 0.95, 1.00)
DARK  = (0.04, 0.11, 0.20)

mat_body    = make_mat("Body",    WHITE,           alpha=0.12, metallic=0.9, roughness=0.05)
mat_shelf   = make_mat("Shelf",   (0.6, 0.8, 1.0), alpha=0.35, metallic=0.0, roughness=0.05)
mat_cam1    = make_mat("Cam1",    TEAL,  metallic=0.8, roughness=0.2, emission=TEAL)
mat_cam2    = make_mat("Cam2",    BLUE,  metallic=0.8, roughness=0.2, emission=BLUE)
mat_cam3    = make_mat("Cam3",    RED,   metallic=0.8, roughness=0.2, emission=RED)
mat_cone1   = make_mat("Cone1",   TEAL,  alpha=0.07)
mat_cone2   = make_mat("Cone2",   BLUE,  alpha=0.07)
mat_cone3   = make_mat("Cone3",   RED,   alpha=0.07)
mat_floor   = make_mat("Floor",   DARK,  metallic=0.0, roughness=0.9)

food_mats = {
    "Milk":         make_mat("Milk",        (0.95, 0.95, 0.97), roughness=0.3),
    "Chicken":      make_mat("Chicken",     (0.85, 0.55, 0.35), roughness=0.9),
    "Broccoli":     make_mat("Broccoli",    (0.15, 0.60, 0.20), roughness=0.9),
    "Strawberries": make_mat("Strawberry",  (0.85, 0.10, 0.15), roughness=0.8),
    "Cheese":       make_mat("Cheese",      (0.95, 0.78, 0.15), roughness=0.7),
    "Leftovers":    make_mat("Leftovers",   (0.45, 0.65, 0.88), roughness=0.6),
    "Yogurt":       make_mat("Yogurt",      (0.90, 0.90, 0.88), roughness=0.4),
    "Salmon":       make_mat("Salmon",      (0.95, 0.45, 0.35), roughness=0.8),
}


def apply_mat(obj, mat):
    if obj.data.materials:
        obj.data.materials[0] = mat
    else:
        obj.data.materials.append(mat)


# ──────────────────────────────────────────────
# 3. FRIDGE DIMENSIONS
# ──────────────────────────────────────────────
FW = 0.70   # width  (x)
FD = 0.70   # depth  (y)
FH = 1.80   # height (z)
T  = 0.025  # wall thickness

hw = FW / 2
hd = FD / 2
hh = FH / 2


def make_wall(name, loc, sx, sy, sz, mat):
    bpy.ops.mesh.primitive_cube_add(location=loc)
    o = bpy.context.active_object
    o.name = name
    o.scale = (sx, sy, sz)
    apply_mat(o, mat)
    link(o)
    return o


# Back wall
make_wall("Wall_Back",   (0,   hd,    hh),       hw,       T/2,  hh,  mat_body)
# Left wall
make_wall("Wall_Left",   (-hw, 0,     hh),        T/2,      hd,   hh,  mat_body)
# Right wall
make_wall("Wall_Right",  (hw,  0,     hh),        T/2,      hd,   hh,  mat_body)
# Bottom
make_wall("Wall_Bottom", (0,   0,     T/2),       hw,       hd,   T/2, mat_body)
# Top
make_wall("Wall_Top",    (0,   0,     FH - T/2),  hw,       hd,   T/2, mat_body)
# (No front wall — open for visibility)

# ──────────────────────────────────────────────
# 4. SHELVES
# ──────────────────────────────────────────────
SHELF_Z = [0.45, 0.90, 1.35]  # metres from floor

for i, z in enumerate(SHELF_Z):
    bpy.ops.mesh.primitive_cube_add(location=(0, 0, z))
    s = bpy.context.active_object
    s.name = f"Shelf_{i+1}"
    s.scale = (hw - T, hd - T, 0.006)
    apply_mat(s, mat_shelf)
    link(s)

# ──────────────────────────────────────────────
# 5. CAMERAS (bodies + lenses)
# ──────────────────────────────────────────────
CAM_SIZE  = 0.030   # body half-size
LENS_R    = 0.010
LENS_D    = 0.018
TOP_Z     = FH - T - CAM_SIZE * 0.7

# Camera positions
cam1_pos = Vector((0.0,          0.0,          TOP_Z))
cam2_pos = Vector((-hw + T + CAM_SIZE, hd - T - CAM_SIZE, TOP_Z))
cam3_pos = Vector(( hw - T - CAM_SIZE, hd - T - CAM_SIZE, TOP_Z))

cameras = [
    ("Camera_1_TopDown",  cam1_pos, mat_cam1, (0, 0, 0)),
    ("Camera_2_CornerL",  cam2_pos, mat_cam2, (0, 0, math.radians(-45))),
    ("Camera_3_CornerR",  cam3_pos, mat_cam3, (0, 0, math.radians( 45))),
]

for name, pos, mat, rot in cameras:
    # Body
    bpy.ops.mesh.primitive_cube_add(location=pos)
    body = bpy.context.active_object
    body.name = name
    body.scale = (CAM_SIZE, CAM_SIZE, CAM_SIZE * 0.55)
    body.rotation_euler = rot
    apply_mat(body, mat)
    link(body)

    # Lens (cylinder pointing down)
    lens_loc = Vector(pos) + Vector((0, 0, -(CAM_SIZE * 0.55 + LENS_D / 2)))
    bpy.ops.mesh.primitive_cylinder_add(
        radius=LENS_R, depth=LENS_D, location=lens_loc
    )
    lens = bpy.context.active_object
    lens.name = f"{name}_Lens"
    apply_mat(lens, mat)
    link(lens)

# ──────────────────────────────────────────────
# 6. FOV CONES
# ──────────────────────────────────────────────
CONE_H = FH - T * 2   # reaches floor from top

def make_cone(name, apex, target_xy_offset, tilt_rx, tilt_ry, rot_rz, r1, mat):
    cx = apex.x + target_xy_offset[0] * 0.5
    cy = apex.y + target_xy_offset[1] * 0.5
    cz = apex.z - CONE_H * 0.5
    bpy.ops.mesh.primitive_cone_add(
        radius1=r1, radius2=0.0, depth=CONE_H,
        location=(cx, cy, cz)
    )
    cone = bpy.context.active_object
    cone.name = name
    cone.rotation_euler = (tilt_rx, tilt_ry, rot_rz)
    apply_mat(cone, mat)
    link(cone)
    return cone


# Cam1 — straight down, wide
make_cone("FOV_Cam1", cam1_pos, (0, 0),
          0, 0, 0,
          r1=0.30, mat=mat_cone1)

# Cam2 — left-back corner, angled toward right-front
make_cone("FOV_Cam2", cam2_pos, (0.25, -0.25),
          math.radians(18), math.radians(-18), math.radians(-45),
          r1=0.22, mat=mat_cone2)

# Cam3 — right-back corner, angled toward left-front
make_cone("FOV_Cam3", cam3_pos, (-0.25, -0.25),
          math.radians(18), math.radians( 18), math.radians( 45),
          r1=0.22, mat=mat_cone3)

# ──────────────────────────────────────────────
# 7. FOOD ITEMS
# ──────────────────────────────────────────────
# (name, location, scale, mat_key)
food_items = [
    # Shelf 1 (z=0.45)
    ("Milk",         (-0.18,  0.08,  0.48 + 0.06),  (0.050, 0.050, 0.120), "Milk"),
    ("Chicken",      ( 0.08,  0.05,  0.45 + 0.025), (0.090, 0.120, 0.025), "Chicken"),
    ("Broccoli",     (-0.04, -0.15,  0.45 + 0.060), (0.055, 0.055, 0.060), "Broccoli"),
    # Shelf 2 (z=0.90)
    ("Strawberries", ( 0.15, -0.12,  0.90 + 0.025), (0.080, 0.060, 0.025), "Strawberries"),
    ("Cheese",       (-0.14,  0.06,  0.90 + 0.035), (0.075, 0.055, 0.035), "Cheese"),
    ("Yogurt",       ( 0.00,  0.10,  0.90 + 0.045), (0.045, 0.045, 0.045), "Yogurt"),
    # Shelf 3 (z=1.35)
    ("Leftovers",    ( 0.05,  0.08,  1.35 + 0.030), (0.090, 0.090, 0.030), "Leftovers"),
    ("Salmon",       (-0.12, -0.05,  1.35 + 0.020), (0.100, 0.060, 0.020), "Salmon"),
]

for fname, floc, fscale, fmat_key in food_items:
    bpy.ops.mesh.primitive_cube_add(location=floc)
    food = bpy.context.active_object
    food.name = f"Food_{fname}"
    food.scale = fscale
    apply_mat(food, food_mats[fmat_key])
    link(food)

# ──────────────────────────────────────────────
# 8. FLOOR PLANE
# ──────────────────────────────────────────────
bpy.ops.mesh.primitive_plane_add(size=4.0, location=(0, 0, 0))
floor = bpy.context.active_object
floor.name = "Floor"
apply_mat(floor, mat_floor)
link(floor)

# ──────────────────────────────────────────────
# 9. TEXT LABELS
# ──────────────────────────────────────────────
labels = [
    ("CAM 1\nTop-Down",   cam1_pos + Vector(( 0.00,  -0.12,  0.06)), TEAL),
    ("CAM 2\nCorner L",   cam2_pos + Vector((-0.14,   0.00,  0.06)), BLUE),
    ("CAM 3\nCorner R",   cam3_pos + Vector(( 0.14,   0.00,  0.06)), RED),
]

for txt, loc, color in labels:
    bpy.ops.object.text_add(location=loc)
    t = bpy.context.active_object
    t.name = f"Label_{txt[:5]}"
    t.data.body = txt
    t.data.size = 0.028
    t.data.align_x = 'CENTER'
    t.rotation_euler = (math.radians(90), 0, 0)
    lmat = make_mat(f"LMat_{txt[:4]}", color, emission=color)
    apply_mat(t, lmat)
    link(t)

# ──────────────────────────────────────────────
# 10. LIGHTING
# ──────────────────────────────────────────────
# Key light — front-left
bpy.ops.object.light_add(type='AREA', location=(-1.2, -1.6, 2.2))
key = bpy.context.active_object
key.name = "Light_Key"
key.data.energy = 350
key.data.size = 1.8
key.rotation_euler = (math.radians(55), 0, math.radians(-30))
link(key)

# Fill light — right
bpy.ops.object.light_add(type='AREA', location=(1.4, 0.2, 1.4))
fill = bpy.context.active_object
fill.name = "Light_Fill"
fill.data.energy = 120
fill.data.size = 1.2
fill.rotation_euler = (math.radians(30), math.radians(45), 0)
link(fill)

# Rim light — back-top (cold blue tint)
bpy.ops.object.light_add(type='AREA', location=(0, 1.2, 2.4))
rim = bpy.context.active_object
rim.name = "Light_Rim"
rim.data.energy = 180
rim.data.color = (0.6, 0.8, 1.0)
rim.data.size = 1.0
rim.rotation_euler = (math.radians(-40), 0, 0)
link(rim)

# ──────────────────────────────────────────────
# 11. SCENE CAMERA
# ──────────────────────────────────────────────
bpy.ops.object.camera_add(location=(1.35, -1.55, 1.15))
scene_cam = bpy.context.active_object
scene_cam.name = "Scene_Camera"
scene_cam.rotation_euler = (math.radians(72), 0, math.radians(42))
scene_cam.data.lens = 35
bpy.context.scene.camera = scene_cam
link(scene_cam)

# ──────────────────────────────────────────────
# 12. WORLD BACKGROUND
# ──────────────────────────────────────────────
world = bpy.context.scene.world
world.use_nodes = True
bg_node = world.node_tree.nodes["Background"]
bg_node.inputs["Color"].default_value    = (0.027, 0.051, 0.102, 1.0)  # dark navy
bg_node.inputs["Strength"].default_value = 0.25

# ──────────────────────────────────────────────
# 13. RENDER SETTINGS
# ──────────────────────────────────────────────
scene = bpy.context.scene
scene.render.engine = 'BLENDER_EEVEE'
scene.render.resolution_x = 1920
scene.render.resolution_y = 1080
scene.render.film_transparent = False

# EEVEE — enable bloom + shadows
if hasattr(scene.eevee, 'use_bloom'):
    scene.eevee.use_bloom = True
    scene.eevee.bloom_intensity = 0.3
    scene.eevee.bloom_threshold = 0.8

# ──────────────────────────────────────────────
# DONE
# ──────────────────────────────────────────────
obj_count = len(fridge_col.objects)
print(f"\n✓ FridgeAI model built — {obj_count} objects in collection 'FridgeAI'")
print("  Cameras:  CAM1 (teal/top), CAM2 (blue/left), CAM3 (red/right)")
print("  Shelves:  3 glass shelves at 0.45 m, 0.90 m, 1.35 m")
print("  Food:     8 sample items across all shelves")
print("  Render:   EEVEE, 1920×1080")
print("\n  Tip: press Numpad-0 to look through Scene_Camera, then F12 to render.")
