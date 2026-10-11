#!/usr/bin/env python3
"""
Generates a 10m x 8m indoor environment for nav2 + obstacle-avoidance testing:
  - Corridor running east-west through the middle
  - Room A (top-left, sparse obstacles)
  - Room B (top-right, cluttered obstacles)
  - Room C (bottom, large room containing the target bottle)
  - Doorways connecting each room to the corridor
Walls are simple boxes (fast collision). Output: world.sdf
"""
import math

WALL_H = 2.5
WALL_T = 0.15
walls = []   # (cx, cy, length, angle_deg)  length is wall's own X extent before rotation

def wall_x(cx, cy, length):   # wall running along world X
    walls.append((cx, cy, length, 0))

def wall_y(cx, cy, length):   # wall running along world Y
    walls.append((cx, cy, length, 90))

# ---------- Perimeter (10 x 8, centered at origin) ----------
wall_x(0, 4.0, 10.0); wall_x(0, -4.0, 10.0)
wall_y(-5.0, 0.0, 8.0); wall_y(5.0, 0.0, 8.0)

# ---------- Corridor walls (corridor is y in [-0.9, 0.9], full x span) ----------
# North corridor wall at y=0.9, with doorway gaps into Room A (left) and Room B (right)
# segments: x -5..-2.6 (gap 1.0 at -2.1..-1.1 to RoomA... let's define doors explicitly)
# South corridor wall at y=-0.9, one doorway into Room C near x=0

DOOR = 1.1  # doorway width

# North wall (y=0.9): door into Room A centered x=-3.2, door into Room B centered x=2.0
# North corridor wall: two doors (into Room A near x=-3.2, into Room B near x=2.0)
bounds = [-5.0, -3.2-DOOR/2, -3.2+DOOR/2, 2.0-DOOR/2, 2.0+DOOR/2, 5.0]
for i in range(0, len(bounds)-1, 2):
    a, b = bounds[i], bounds[i+1]
    if b - a > 0.05:
        wall_x((a+b)/2, 0.9, b-a)

# South corridor wall: one door into Room C near x=0.0
bounds = [-5.0, -DOOR/2, DOOR/2, 5.0]
for i in range(0, len(bounds)-1, 2):
    a, b = bounds[i], bounds[i+1]
    if b - a > 0.05:
        wall_x((a+b)/2, -0.9, b-a)

# ---------- Divider between Room A and Room B (vertical wall at x=-0.5, y 0.9..4.0) ----------
wall_y(-0.5, (0.9+4.0)/2, 4.0-0.9)

# ================= Obstacles =================
# (type, x, y, yaw_deg, extra...)  boxes: (sx,sy,sz)  cylinders: (radius, length)
boxes = []
cylinders = []

# --- Room A (top-left, x -5..-0.5, y 0.9..4.0) : SPARSE ---
boxes.append((-3.8, 2.0, 0, 0.6, 0.6, 0.8))
cylinders.append((-2.2, 3.2, 20, 0.25, 1.0))
boxes.append((-1.3, 1.6, 15, 0.5, 0.9, 0.5))

# --- Room B (top-right, x -0.5..5, y 0.9..4.0) : CLUTTERED ---
boxes.append((0.6, 1.6, 0, 0.5, 0.5, 0.6))
boxes.append((1.6, 2.6, 30, 0.7, 0.4, 0.9))
cylinders.append((2.6, 1.5, 0, 0.2, 1.2))
cylinders.append((3.6, 3.2, 0, 0.3, 0.7))
boxes.append((4.2, 1.8, -20, 0.5, 0.5, 0.5))
boxes.append((0.8, 3.3, 10, 0.4, 0.4, 1.0))
cylinders.append((3.0, 2.2, 0, 0.18, 0.9))

# --- Corridor: a couple of mild obstacles to force avoidance while traversing ---
boxes.append((-1.5, 0.3, 0, 0.4, 0.4, 0.6))
cylinders.append((1.0, -0.3, 0, 0.2, 1.0))

# --- Room C (bottom, large, x -5..5, y -4..-0.9): contains target bottle ---
boxes.append((-3.5, -2.5, 0, 0.6, 1.2, 0.75))   # table-like block
boxes.append((-3.5, -3.3, 0, 0.5, 0.5, 0.75))
cylinders.append((-1.0, -1.8, 0, 0.25, 1.0))
boxes.append((1.5, -2.0, 25, 0.8, 0.5, 0.6))
cylinders.append((3.0, -3.0, 0, 0.3, 0.8))
boxes.append((3.5, -1.5, 0, 0.5, 0.5, 0.5))
boxes.append((-0.5, -3.5, 0, 0.9, 0.4, 0.5))

BOTTLE_POS = (2.2, -2.4, 0.001)

# ================= SDF generation =================
def wall_sdf(i, cx, cy, length, angle_deg):
    yaw = math.radians(angle_deg)
    return f"""
    <model name="wall_{i}">
      <static>true</static>
      <pose>{cx:.3f} {cy:.3f} {WALL_H/2:.3f} 0 0 {yaw:.4f}</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{length:.3f} {WALL_T} {WALL_H}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{length:.3f} {WALL_T} {WALL_H}</size></box></geometry>
          <material>
            <ambient>0.78 0.76 0.70 1</ambient>
            <diffuse>0.82 0.80 0.74 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>"""

def box_sdf(i, cx, cy, yaw_deg, sx, sy, sz):
    yaw = math.radians(yaw_deg)
    colors = ["0.55 0.35 0.2 1", "0.6 0.5 0.35 1", "0.45 0.45 0.48 1", "0.5 0.3 0.25 1"]
    c = colors[i % len(colors)]
    return f"""
    <model name="box_{i}">
      <static>true</static>
      <pose>{cx:.3f} {cy:.3f} {sz/2:.3f} 0 0 {yaw:.4f}</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
          <material><ambient>{c}</ambient><diffuse>{c}</diffuse></material>
        </visual>
      </link>
    </model>"""

def cyl_sdf(i, cx, cy, yaw_deg, radius, length):
    colors = ["0.3 0.3 0.32 1", "0.5 0.15 0.1 1", "0.2 0.4 0.45 1"]
    c = colors[i % len(colors)]
    return f"""
    <model name="cyl_{i}">
      <static>true</static>
      <pose>{cx:.3f} {cy:.3f} {length/2:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><cylinder><radius>{radius:.3f}</radius><length>{length:.3f}</length></cylinder></geometry>
        </collision>
        <visual name="visual">
          <geometry><cylinder><radius>{radius:.3f}</radius><length>{length:.3f}</length></cylinder></geometry>
          <material><ambient>{c}</ambient><diffuse>{c}</diffuse></material>
        </visual>
      </link>
    </model>"""

parts = []
for i, (cx, cy, length, ang) in enumerate(walls):
    parts.append(wall_sdf(i, cx, cy, length, ang))
for i, b in enumerate(boxes):
    parts.append(box_sdf(i, *b))
for i, cyl in enumerate(cylinders):
    parts.append(cyl_sdf(i, *cyl))

floor = f"""
    <model name="floor">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>10.3 8.3 0.1</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>10.3 8.3 0.1</size></box></geometry>
          <material>
            <ambient>0.5 0.5 0.52 1</ambient>
            <diffuse>0.55 0.55 0.57 1</diffuse>
            <pbr><metal><roughness>0.8</roughness></metal></pbr>
          </material>
        </visual>
      </link>
      <pose>0 0 -0.05 0 0 0</pose>
    </model>"""

bottle_include = f"""
    <include>
      <uri>model://bottle</uri>
      <name>target_bottle</name>
      <pose>{BOTTLE_POS[0]} {BOTTLE_POS[1]} {BOTTLE_POS[2]} 0 0 0</pose>
    </include>"""

world = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="nav_world">

    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"></plugin>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"></plugin>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"></plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"></plugin>

    <scene>
      <ambient>0.5 0.5 0.5 1</ambient>
      <background>0.6 0.7 0.8 1</background>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.85 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>-0.4 0.2 -0.9</direction>
    </light>

    <light type="point" name="room_a_light">
      <pose>-3.0 2.3 2.3 0 0 0</pose>
      <diffuse>0.9 0.9 0.8 1</diffuse>
      <attenuation><range>8</range><constant>0.3</constant><linear>0.2</linear><quadratic>0.01</quadratic></attenuation>
      <cast_shadows>false</cast_shadows>
    </light>
    <light type="point" name="room_b_light">
      <pose>2.0 2.3 2.3 0 0 0</pose>
      <diffuse>0.9 0.9 0.8 1</diffuse>
      <attenuation><range>8</range><constant>0.3</constant><linear>0.2</linear><quadratic>0.01</quadratic></attenuation>
      <cast_shadows>false</cast_shadows>
    </light>
    <light type="point" name="room_c_light">
      <pose>0.0 -2.5 2.3 0 0 0</pose>
      <diffuse>0.9 0.9 0.8 1</diffuse>
      <attenuation><range>10</range><constant>0.3</constant><linear>0.15</linear><quadratic>0.01</quadratic></attenuation>
      <cast_shadows>false</cast_shadows>
    </light>

    {floor}
    {''.join(parts)}
    {bottle_include}

  </world>
</sdf>
"""

with open("world.sdf", "w") as f:
    f.write(world)
print("walls:", len(walls), "boxes:", len(boxes), "cylinders:", len(cylinders))
print("bottle at", BOTTLE_POS)
