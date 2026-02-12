extends Node

func _ready() -> void:
	if not ClassDB.class_exists(&"SolideanHelper"):
		OS.alert(
			"Please ensure these DLLs are in the same folder as the executable:\n"
			+"- solideanGodotBridge.dll\n"
			+"- solidean.dll",
			"Solidean - Hello Godot"
		)
		get_tree().quit(1)
