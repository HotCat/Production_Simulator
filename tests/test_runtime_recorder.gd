extends SceneTree


func _initialize() -> void:
	var recorder: Node = root.get_node_or_null("RuntimeRecorder") as Node
	var settings_ok: bool = (
		ProjectSettings.get_setting("runtime_recorder/codec", "") == "h264"
		and int(ProjectSettings.get_setting("runtime_recorder/fps", 0)) == 30
		and int(ProjectSettings.get_setting("runtime_recorder/width", 0)) == 1280
		and int(ProjectSettings.get_setting("runtime_recorder/height", 0)) == 720
	)
	var recorder_ok: bool = recorder != null and not recorder.is_recording and not recorder.is_finalizing
	var ffmpeg_available: bool = recorder != null and not str(recorder.call("_find_ffmpeg")).is_empty()
	print("Runtime recorder autoload: ", recorder != null)
	print("Runtime recorder FFmpeg available: ", ffmpeg_available)
	if not settings_ok or not recorder_ok or not ffmpeg_available:
		push_error("Runtime recorder configuration verification failed")
		quit(1)
	else:
		print("Runtime recorder configuration verification passed")
		quit(0)
