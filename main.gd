extends Node3D

## Sculpture demo scene - boolean operations + particle creation

# === Node References ===
@onready var solidean_helper := SolideanHelper.new()
@onready var sculpture := $Sculpture
@onready var tool := $Tool
@onready var sculpture_stats := $UI/MarginContainerBL/Container/StatsStatue
@onready var particles_stats := $UI/MarginContainerBL/Container/Stats
@onready var sculpture_wireframe := $UI/MarginContainerBL/Container/WireframeStatue
@onready var particles_wireframe := $UI/MarginContainerBL/Container/Wireframe
@onready var camera: Camera3D = $Camera3D
@onready var model_reset: Button = $UI/MarginContainerTR/Container/ResetButton

# === Configuration ===
@export_range(8, 64) var sphere_radial_segments = 32:
	set(value):
		sphere_radial_segments = value
		if tool: tool.mesh.radial_segments = value
@export_range(4, 32) var sphere_rings := 16:
	set(value):
		sphere_rings = value
		if tool: tool.mesh.rings = value
@export var tool_min_scale := 0.1
@export var tool_max_scale := 2.0

# === Collisions ===
# Whether to use trimesh and multiple convex collision shapes or single convex shapes
# Huge timing/quality differences
var high_quality_collision_shapes_static := false
var high_quality_collision_shapes_rigid := false

# === Constants ===
const RAYCAST_DISTANCE := 100.0
const TOOL_SCALE_SPEED := 0.1

const PARTICLE_IMPULSE_STRENGTH := 5.0
const PARTICLE_COLLISION_DELAY := 0.0 # Seconds before enabling particle collisions
const PHYSICS_LAYER_SCULPTURE := 2 # Layer 3 (1 << 2)
const SCULPT_HOLD_INTERVAL := 0.2


# === Materials ===
@onready var sculpture_material: StandardMaterial3D = sculpture.get_active_material(0)
@onready var particles_material: StandardMaterial3D = sculpture_material.duplicate(true)

# === Raycast State ===
var raycast_point := Vector3.INF
var raycast_surface_normal := Vector3.INF

# === Hold-to-sculpt ===
var _lmb_held := false
var _sculpt_timer := 0.0

# === Statistics ===
var particles_vertices := 0
var particles_triangles := 0
var sculpture_vertices := 0
var sculpture_triangles := 0

# === Timings (ms) ===
var timing_intersection_ms := 0.0
var timing_intersection_import_ms := 0.0
var timing_intersection_execute_ms := 0.0
var timing_intersection_export_ms := 0.0
var timing_difference_ms := 0.0
var timing_difference_import_ms := 0.0
var timing_difference_execute_ms := 0.0
var timing_difference_export_ms := 0.0
var timing_rigid_body_ms := 0.0
var timing_collision_ms := 0.0


func _exit_tree() -> void:
	if solidean_helper:
		solidean_helper.free()


func _ready() -> void:
	# Configure tool mesh resolution
	var sphere := tool.mesh as SphereMesh
	sphere.radial_segments = sphere_radial_segments
	sphere.rings = sphere_rings

	tool.scale = Vector3.ONE * (tool_max_scale + tool_min_scale) / 3
	
	# Initial state: wireframes off
	_on_wireframe_toggled(false)
	_on_wireframe_toggled_particles(false)

	solidean_helper.set_normal_split_angle(60.0)

	model_reset.pressed.connect(_load_current_model)
	_load_current_model()
	
	
	# To distinguish particles from model
	particles_material.albedo_color *= 0.8

	camera.jump_to_target()

	update_ui()

func _clear_particles() -> void:
	"""Removes all particle rigid bodies from the scene"""
	for child in get_children():
		if child is RigidBody3D:
			# Clear cached mesh data before freeing to avoid dangling pointers
			for rb_child in child.get_children():
				if rb_child is MeshInstance3D:
					solidean_helper.clear_cache_entry(rb_child)
			child.queue_free()
	particles_vertices = 0
	particles_triangles = 0


func _load_current_model() -> void:
	"""Resets sculpture to initial state"""
	_clear_particles()

	var model_path := "res://assets/cemetery_statue.obj"

	var resource = load(model_path)
	# print("Loaded resource type: ", typeof(resource), " - ", resource)

	var mesh := resource as Mesh
	if mesh:
		sculpture.mesh = mesh
		# Initial self-union to prepare sculpture
		# clear_a=true since mesh changed
		solidean_helper.operation(sculpture, null, SolideanHelper.UNION, true, false, null, SolideanHelper.SUPERSOLID)
		sculpture_vertices = solidean_helper.getNumVerticesLastOperation()
		sculpture_triangles = solidean_helper.getNumTrianglesLastOperation()
		_recreate_collision()
		update_ui()
		
		# Optionally export the mesh as an .obj
		# solidean_helper.export_to_obj(sculpture, "solidean-result.obj")

func _color_timing(ms: float, interval_width = 5.0) -> String:
	"""Returns color name based on timing value (0-5ms: light green, 5-10ms: yellow, 10ms+: red)"""
	if ms <= interval_width:
		return "light_green"
	elif ms <= interval_width * 2:
		return "light_yellow"
	else:
		return "#FFCCCB"


func _process(delta: float) -> void:
	if timing_intersection_ms > 0:
		$UI/MarginContainerTL/Container/Timings.text = "[u]Timings[/u] \n" \
		 +"Intersection: [color=%s]%.2f[/color] ms\n(import: [color=%s]%.2f[/color] ms, [color=#6930FF]solidean[/color]: [color=%s]%.2f[/color] ms, export: [color=%s]%.2f[/color] ms)\n" \
				% [_color_timing(timing_intersection_ms, 10), timing_intersection_ms,
				   _color_timing(timing_intersection_import_ms), timing_intersection_import_ms,
				   _color_timing(timing_intersection_execute_ms), timing_intersection_execute_ms,
				   _color_timing(timing_intersection_export_ms), timing_intersection_export_ms] \
			+"Difference: [color=%s]%.2f[/color] ms\n(import: [color=%s]%.2f[/color] ms, [color=#6930FF]solidean[/color]: [color=%s]%.2f[/color] ms, export: [color=%s]%.2f[/color] ms)\n" \
				% [_color_timing(timing_difference_ms, 10), timing_difference_ms,
				   _color_timing(timing_difference_import_ms), timing_difference_import_ms,
				   _color_timing(timing_difference_execute_ms), timing_difference_execute_ms,
				   _color_timing(timing_difference_export_ms), timing_difference_export_ms] \
			+"Rigid body creation: [color=%s]%.2f[/color] ms\n" \
				% [_color_timing(timing_rigid_body_ms, 10), timing_rigid_body_ms] \
			+"Static body creation: [color=%s]%.2f[/color] ms" \
				% [_color_timing(timing_collision_ms, 10), timing_collision_ms]

	$UI/MarginContainerTR/Container/FPS.text = "FPS: %s\n" % Engine.get_frames_per_second()

	# Hold-to-sculpt: repeat every SCULPT_HOLD_INTERVAL while LMB is held
	if _lmb_held and tool.visible:
		_sculpt_timer += delta
		if _sculpt_timer >= SCULPT_HOLD_INTERVAL:
			_sculpt_timer -= SCULPT_HOLD_INTERVAL
			_perform_sculpt_operation()

func _physics_process(_delta: float) -> void:
	_update_tool_position()


func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventKey and event.pressed and event.keycode == KEY_ESCAPE:
		get_tree().quit()
		return
	if event is InputEventMouseButton:
		if event.button_index == MOUSE_BUTTON_LEFT:
			_lmb_held = event.pressed
			_sculpt_timer = 0.0
		if event.pressed:
			_handle_mouse_input(event)


# === Tool Positioning ===

func _update_tool_position() -> void:
	"""Updates tool position via raycast to sculpture surface"""
	raycast_point = Vector3.INF
	raycast_surface_normal = Vector3.INF
	
	# Hide tool when camera is orbiting
	if Input.mouse_mode == Input.MOUSE_MODE_CAPTURED:
		tool.visible = false
		return
	
	# var camera := get_viewport().get_camera_3d()
	var mouse_pos := get_viewport().get_mouse_position()
	var origin := camera.project_ray_origin(mouse_pos)
	var direction := camera.project_ray_normal(mouse_pos)
	
	# Raycast against sculpture (layer 2)
	var query := PhysicsRayQueryParameters3D.create(
		origin,
		origin + direction * RAYCAST_DISTANCE
	)
	query.collision_mask = 1 << PHYSICS_LAYER_SCULPTURE
	
	var result := get_world_3d().direct_space_state.intersect_ray(query)
	
	if result:
		raycast_point = result.position
		raycast_surface_normal = result.normal
		tool.position = raycast_point
		tool.visible = true
	else:
		tool.visible = false


# === Input Handling ===

func _handle_mouse_input(event: InputEventMouseButton) -> void:
	"""Handles mouse wheel (tool scaling) and left click (sculpting)"""
	var is_shift_held := Input.is_key_pressed(KEY_SHIFT)
	
	# Shift + Scroll: Resize tool
	if is_shift_held:
		if event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			tool.scale -= TOOL_SCALE_SPEED * Vector3.ONE
		elif event.button_index == MOUSE_BUTTON_WHEEL_UP:
			tool.scale += TOOL_SCALE_SPEED * Vector3.ONE
		
		tool.scale = tool.scale.clamp(
			Vector3.ONE * tool_min_scale,
			Vector3.ONE * tool_max_scale
		)
	
	# Left Click: Sculpt (create particle and subtract from sculpture)
	if event.button_index == MOUSE_BUTTON_LEFT and tool.visible:
		_perform_sculpt_operation()


# === Sculpting Operations ===

func _perform_sculpt_operation() -> void:
	"""Creates particle from intersection and subtracts tool from sculpture"""
	var t_start: int

	# Intersection operation
	t_start = Time.get_ticks_usec()
	var particle_mesh := MeshInstance3D.new()
	solidean_helper.operation(
		sculpture,
		tool ,
		SolideanHelper.INTERSECTION,
		false,
		false, # Don't clear tool mesh yet (reused for difference)
		particle_mesh,
		SolideanHelper.SUPERSOLID,
		true # Cache triangle data for tool (geometry doesn't change, only transform)
	)
	timing_intersection_ms = (Time.get_ticks_usec() - t_start) / 1000.0
	timing_intersection_import_ms = solidean_helper.getImportTimeMs()
	timing_intersection_execute_ms = solidean_helper.getExecuteTimeMs()
	timing_intersection_export_ms = solidean_helper.getExportTimeMs()

	# Only create particle if intersection is non-empty
	if particle_mesh.mesh != null and particle_mesh.mesh.get_surface_count() > 0:
		t_start = Time.get_ticks_usec()
		var rigid_body := _create_particle_rigid_body(particle_mesh)
		timing_rigid_body_ms = (Time.get_ticks_usec() - t_start) / 1000.0

		if rigid_body:
			add_child(rigid_body)
			particles_vertices += solidean_helper.getNumVerticesLastOperation()
			particles_triangles += solidean_helper.getNumTrianglesLastOperation()
		else:
			particle_mesh.queue_free()
	else:
		particle_mesh.queue_free()

	# Difference operation
	t_start = Time.get_ticks_usec()
	solidean_helper.operation(
		sculpture,
		tool ,
		SolideanHelper.DIFFERENCE,
		false,
		true, # Clear tool mesh (will be reimported on next operation)
		null,
		SolideanHelper.SUPERSOLID,
		true # Cache triangle data for tool (geometry doesn't change, only transform)
	)
	timing_difference_ms = (Time.get_ticks_usec() - t_start) / 1000.0
	timing_difference_import_ms = solidean_helper.getImportTimeMs()
	timing_difference_execute_ms = solidean_helper.getExecuteTimeMs()
	timing_difference_export_ms = solidean_helper.getExportTimeMs()

	# Recreate collision
	t_start = Time.get_ticks_usec()
	_recreate_collision()
	timing_collision_ms = (Time.get_ticks_usec() - t_start) / 1000.0

	sculpture_vertices = solidean_helper.getNumVerticesLastOperation()
	sculpture_triangles = solidean_helper.getNumTrianglesLastOperation()
	update_ui()

func _create_particle_rigid_body(particle_mesh: MeshInstance3D) -> RigidBody3D:
	"""Creates a rigid body with convex collision from a mesh"""
	# particle_mesh.create_multiple_convex_collisions()
	
	# This is MUCH faster - not ideal for concave particles though
	particle_mesh.create_convex_collision(true, false)
	
	# Validate collision was created
	if particle_mesh.get_child_count() < 1:
		push_warning("Failed to create collision for particle mesh")
		return null

	var static_body := particle_mesh.get_child(0) as StaticBody3D
	if not static_body or static_body.get_child_count() < 1:
		push_warning("Invalid collision structure for particle")
		return null

	# Create rigid body and transfer all collision shapes
	var rigid_body := RigidBody3D.new()
	while static_body.get_child_count() > 0:
		var collision_shape := static_body.get_child(0)
		static_body.remove_child(collision_shape)
		rigid_body.add_child(collision_shape)
	
	# Clean up temporary static body
	particle_mesh.remove_child(static_body)
	static_body.queue_free()
	
	# Attach mesh to rigid body
	rigid_body.add_child(particle_mesh)
	
	# Apply initial velocity along surface normal
	rigid_body.linear_velocity = (raycast_surface_normal + Vector3.UP).normalized() * PARTICLE_IMPULSE_STRENGTH
	
	# Apply material
	particle_mesh.mesh.surface_set_material(0, particles_material)
	
	return rigid_body


# === Collision Management ===

func _recreate_collision() -> void:
	"""Recreates trimesh collision for sculpture after boolean operation"""
	# Remove existing collision
	if sculpture.get_child_count() == 1:
		var old_collision := sculpture.get_child(0)
		sculpture.remove_child(old_collision)
		old_collision.queue_free()
	
	# Create new collision (only if sculpture has surfaces)
	if sculpture.get_surface_override_material_count() > 0:
		sculpture.create_trimesh_collision()
		# Faster, but misleading ray intersections
		# sculpture.create_convex_collision()
		
		# Set to physics layer 2 for raycasting
		if sculpture.get_child_count() > 0:
			sculpture.get_child(0).collision_layer |= 1 << PHYSICS_LAYER_SCULPTURE


# === UI ===

func update_ui() -> void:
	"""Updates statistics display"""
	particles_stats.text = "Vertices: %s\nTriangles: %s" % [
		_format_number(particles_vertices),
		_format_number(particles_triangles)
	]
	sculpture_stats.text = "Vertices: %s\nTriangles: %s" % [
		_format_number(sculpture_vertices),
		_format_number(sculpture_triangles)
	]

func _format_number(n: int) -> String:
	"""Formats integer with comma separators (1000 -> 1,000)"""
	var s := str(n)
	var result := ""
	while s.length() > 3:
		result = "," + s.substr(s.length() - 3, 3) + result
		s = s.substr(0, s.length() - 3)
	return s + result


# === Wireframe Toggles ===

func _on_wireframe_toggled(toggled_on: bool) -> void:
	"""Toggle sculpture wireframe visibility"""
	var alpha := 1.0 if toggled_on else 0.0
	
	sculpture_material.next_pass.set_shader_parameter("alpha", alpha)
	sculpture_wireframe.button_pressed = toggled_on

func _on_wireframe_toggled_particles(toggled_on: bool) -> void:
	"""Toggle particles wireframe visibility"""
	var alpha := 1.0 if toggled_on else 0.0
	particles_material.next_pass.set_shader_parameter("alpha", alpha)
	particles_wireframe.button_pressed = toggled_on
