shader_type spatial;
render_mode unshaded, cull_back, wireframe;

uniform vec4 wireframe_color : source_color = vec4(0.0, 0.6, 1.0, 1.0);

uniform float offset = 0.002;

// To toggle wireframe shader
uniform float alpha = 1.0;

void vertex() {
	// This only helps most of the time, some parts of wireframe still clipped
	vec3 view_pos = (VIEW_MATRIX * vec4(VERTEX, 1.0)).xyz;
	view_pos.z += offset;
	VERTEX = (INV_VIEW_MATRIX * vec4(view_pos, 1.0)).xyz;
}

void fragment() {
	vec3 col = wireframe_color.rgb;
	ALBEDO = col;
	ALPHA *= wireframe_color.a * alpha;
}