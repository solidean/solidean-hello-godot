extends Camera3D

## Simple orbital camera that rotates around a target mesh's center

# === Target ===
@export var cam_target: MeshInstance3D
var _target := Vector3.ZERO
@export var target_smooth_speed := 2.0
@export var min_y := 0.2

# === Zoom ===
@export_range(0.1, 10.0) var min_zoom := 1.0
@export_range(0.1, 20.0) var max_zoom := 5.5
@export var zoom_step := 0.5
@export var zoom_smooth_speed := 10.0
var _current_zoom := 0.0
var _target_zoom := 0.0

# === Rotation ===
@export var sensitivity := 0.002
@export_range(0.0, 45.0) var pole_limit_degrees := 5.0
var _angle_phi := 1.0
var _angle_theta := 1.0

# === State ===
var _mmb_held := false
var _saved_mouse_pos := Vector2()


func _ready() -> void:
	# Don't inherit potential parent transformations
	set_as_top_level(true)
	
	# Initialize zoom
	_target_zoom = max_zoom * 0.7
	_current_zoom = _target_zoom

	jump_to_target()

func jump_to_target() -> void:
	if not is_instance_valid(cam_target):
		push_warning("Camera requires Cam Target")
	else:
		_target = cam_target.get_aabb().get_center() + cam_target.global_position

func _unhandled_input(event: InputEvent) -> void:
	# Zoom (disabled when shift is held for other tools)
	if event is InputEventMouseButton and not Input.is_key_pressed(KEY_SHIFT):
		if event.button_index == MOUSE_BUTTON_WHEEL_UP:
			_target_zoom = clamp(_target_zoom - zoom_step, min_zoom, max_zoom)
		elif event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			_target_zoom = clamp(_target_zoom + zoom_step, min_zoom, max_zoom)
	
	# Orbit with middle mouse button
	if event is InputEventMouseButton and event.button_index == MOUSE_BUTTON_MIDDLE:
		if event.is_pressed():
			_saved_mouse_pos = get_viewport().get_mouse_position()
			Input.set_mouse_mode(Input.MOUSE_MODE_CAPTURED)
			_mmb_held = true
		elif event.is_released():
			Input.set_mouse_mode(Input.MOUSE_MODE_VISIBLE)
			get_viewport().warp_mouse(_saved_mouse_pos)
			_mmb_held = false
	
	# Mouse motion for orbiting
	if _mmb_held and event is InputEventMouseMotion:
		# Horizontal rotation (phi)
		_angle_phi = fmod(_angle_phi + event.relative.x * sensitivity, TAU)
		
		# Vertical rotation (theta) - clamped to avoid poles
		var pole_limit_rad := deg_to_rad(pole_limit_degrees)
		_angle_theta = clamp(
			_angle_theta - event.relative.y * sensitivity,
			pole_limit_rad,
			PI - pole_limit_rad
		)


func _process(delta: float) -> void:
	# Null check
	if not is_instance_valid(cam_target):
		return
	
	# Smoothly follow target mesh center
	var target_center := cam_target.get_aabb().get_center() + cam_target.global_position
	_target = _target.lerp(target_center, delta * target_smooth_speed)
	
	# Smooth zoom
	_current_zoom = lerp(_current_zoom, _target_zoom, delta * zoom_smooth_speed)
	
	# Calculate camera position using spherical coordinates
	var offset := Vector3(
		sin(_angle_theta) * cos(_angle_phi),
		cos(_angle_theta),
		sin(_angle_theta) * sin(_angle_phi)
	) * _current_zoom

	# Ensure camera never goes below y = 0
	var cam_position := _target + offset
	cam_position.y = max(cam_position.y, min_y)

	look_at_from_position(cam_position, _target)
