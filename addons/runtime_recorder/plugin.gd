@tool
extends EditorPlugin

const AUTOLOAD_NAME := "RuntimeRecorder"
const AUTOLOAD_PATH := "res://addons/runtime_recorder/runtime_recorder.gd"
var _added_autoload := false


func _enter_tree() -> void:
	if not ProjectSettings.has_setting("autoload/" + AUTOLOAD_NAME):
		add_autoload_singleton(AUTOLOAD_NAME, AUTOLOAD_PATH)
		_added_autoload = true


func _exit_tree() -> void:
	if _added_autoload:
		remove_autoload_singleton(AUTOLOAD_NAME)
