extends Node
## Runtime viewport recorder.
##
## Captures rendered frames on the render thread, queues them in a bounded
## buffer, and streams RGBA frames asynchronously to a local FFmpeg process.
## The game loop never waits on disk or encoder I/O. Press F9 (or Ctrl+R) to
## toggle recording; the recorder writes an H.264/H.265 MP4 to the configured
## output directory.

signal recording_started(output_path: String)
signal recording_finalizing(output_path: String)
signal recording_stopped(output_path: String)
signal recording_failed(message: String)

const MAX_QUEUED_FRAMES := 12
const HOTKEY_LABEL := "F9 (or Ctrl+R)"

var is_recording := false
var is_finalizing := false
var last_output_path := ""
var dropped_frames := 0

var _capture_interval_usec := 33333
var _next_capture_usec := 0
var _frame_size := Vector2i.ZERO
var _tcp_port := 0
var _ffmpeg_pid := -1
var _worker_thread: Thread
var _queue_mutex := Mutex.new()
var _frame_ready := Semaphore.new()
var _frame_queue: Array[PackedByteArray] = []
var _worker_should_stop := false
var _worker_finished := false
var _worker_error := ""
var _stream_closed := false
var _recording_started_usec := 0
var _indicator_window: Window
var _indicator_label: Label


func _ready() -> void:
	process_mode = Node.PROCESS_MODE_ALWAYS
	RenderingServer.frame_post_draw.connect(_on_frame_post_draw)
	_create_indicator_window()
	print("Runtime recorder ready. Press %s to start/stop recording." % HOTKEY_LABEL)


func _exit_tree() -> void:
	if is_recording:
		_request_stop()
	if _worker_thread != null:
		_worker_thread.wait_to_finish()
		_worker_thread = null
	_tcp_port = 0


func _input(event: InputEvent) -> void:
	if not (event is InputEventKey and event.pressed and not event.echo):
		return
	var key := event as InputEventKey
	var is_f9 := key.keycode == KEY_F9 or key.physical_keycode == KEY_F9
	var is_ctrl_r := key.ctrl_pressed and (key.keycode == KEY_R or key.physical_keycode == KEY_R)
	if not is_f9 and not is_ctrl_r:
		return
	get_viewport().set_input_as_handled()
	if is_recording:
		stop_recording()
	elif not is_finalizing:
		start_recording()


func _process(_delta: float) -> void:
	if is_recording:
		_update_recording_indicator()
	if not is_finalizing:
		return
	var worker_done := false
	_queue_mutex.lock()
	worker_done = _worker_finished
	var worker_error := _worker_error
	_queue_mutex.unlock()
	if worker_done and not _stream_closed:
		if _worker_thread != null:
			_worker_thread.wait_to_finish()
			_worker_thread = null
		_stream_closed = true
		if not worker_error.is_empty():
			_fail(worker_error)
			return
	if _stream_closed and (_ffmpeg_pid <= 0 or not OS.is_process_running(_ffmpeg_pid)):
		_finish_recording()


func start_recording() -> bool:
	if is_recording or is_finalizing:
		return false
	var ffmpeg_path := _find_ffmpeg()
	if ffmpeg_path.is_empty():
		_fail(_ffmpeg_install_hint())
		return false
	var viewport_texture := get_viewport().get_texture()
	if viewport_texture == null:
		_fail("The game viewport could not be captured.")
		return false
	var first_image := viewport_texture.get_image()
	if first_image == null or first_image.is_empty():
		_fail("The game viewport could not be captured.")
		return false
	var requested_width := maxi(2, int(ProjectSettings.get_setting("runtime_recorder/width", 1280))) & ~1
	var requested_height := maxi(2, int(ProjectSettings.get_setting("runtime_recorder/height", 720))) & ~1
	_frame_size = Vector2i(requested_width, requested_height)
	var fps := maxi(1, int(ProjectSettings.get_setting("runtime_recorder/fps", 30)))
	_capture_interval_usec = int(1000000.0 / fps)
	var output_directory := _globalize_directory(str(ProjectSettings.get_setting("runtime_recorder/output_directory", "res://recordings")))
	var directory_error := DirAccess.make_dir_recursive_absolute(output_directory)
	if directory_error != OK and directory_error != ERR_ALREADY_EXISTS:
		_fail("Could not create recording directory: %s" % output_directory)
		return false
	var codec := str(ProjectSettings.get_setting("runtime_recorder/codec", "h264")).to_lower()
	if codec not in ["h264", "h265"]:
		codec = "h264"
	var stamp := Time.get_datetime_string_from_system(false, true).replace(":", "-").replace(" ", "_")
	last_output_path = output_directory.path_join("mg400_simulation_%s_%s.mp4" % [stamp, codec])
	_tcp_port = 38000 + int(Time.get_ticks_usec() % 2000)
	var ffmpeg_args := _build_ffmpeg_args(codec, fps, last_output_path)
	_ffmpeg_pid = OS.create_process(ffmpeg_path, ffmpeg_args, false)
	if _ffmpeg_pid <= 0:
		_fail("FFmpeg could not be started.")
		return false
	_frame_queue.clear()
	_worker_should_stop = false
	_worker_finished = false
	_worker_error = ""
	_stream_closed = false
	dropped_frames = 0
	_worker_thread = Thread.new()
	var thread_error := _worker_thread.start(_frame_writer.bind(_tcp_port))
	if thread_error != OK:
		_fail("The asynchronous frame writer could not start.")
		return false
	is_recording = true
	_recording_started_usec = Time.get_ticks_usec()
	_next_capture_usec = _recording_started_usec
	_show_indicator("● REC  00:00\n%s to stop" % HOTKEY_LABEL, Color(1.0, 0.22, 0.18))
	print("Recording started: ", last_output_path)
	recording_started.emit(last_output_path)
	return true


func stop_recording() -> void:
	if not is_recording:
		return
	_request_stop()
	recording_finalizing.emit(last_output_path)
	_show_indicator("Finalizing MP4…", Color(1.0, 0.72, 0.18))
	print("Recording stopped; finalizing: ", last_output_path)


func _request_stop() -> void:
	is_recording = false
	is_finalizing = true
	_queue_mutex.lock()
	_worker_should_stop = true
	_queue_mutex.unlock()
	_frame_ready.post()


func _on_frame_post_draw() -> void:
	if not is_recording:
		return
	var now := Time.get_ticks_usec()
	if now < _next_capture_usec:
		return
	var frames_due := clampi(int((now - _next_capture_usec) / _capture_interval_usec) + 1, 1, 4)
	_next_capture_usec += frames_due * _capture_interval_usec
	var viewport_texture := get_viewport().get_texture()
	if viewport_texture == null:
		return
	var image := viewport_texture.get_image()
	if image == null or image.is_empty():
		return
	if image.get_width() != _frame_size.x or image.get_height() != _frame_size.y:
		image.resize(_frame_size.x, _frame_size.y, Image.INTERPOLATE_BILINEAR)
	if image.get_format() != Image.FORMAT_RGBA8:
		image.convert(Image.FORMAT_RGBA8)
	var frame_data := image.get_data()
	var queued_count := 0
	_queue_mutex.lock()
	for _frame_index in frames_due:
		if _frame_queue.size() < MAX_QUEUED_FRAMES:
			_frame_queue.push_back(frame_data)
			queued_count += 1
		else:
			dropped_frames += 1
	_queue_mutex.unlock()
	for _frame_index in queued_count:
		_frame_ready.post()


func _frame_writer(port: int) -> void:
	var connect_deadline := Time.get_ticks_msec() + 10000
	var stream: StreamPeerTCP
	while Time.get_ticks_msec() < connect_deadline:
		stream = StreamPeerTCP.new()
		if stream.connect_to_host("127.0.0.1", port) == OK:
			var attempt_deadline := Time.get_ticks_msec() + 500
			while stream.get_status() == StreamPeerTCP.STATUS_CONNECTING and Time.get_ticks_msec() < attempt_deadline:
				stream.poll()
				OS.delay_msec(10)
			if stream.get_status() == StreamPeerTCP.STATUS_CONNECTED:
				break
		OS.delay_msec(25)
	if stream == null or stream.get_status() != StreamPeerTCP.STATUS_CONNECTED:
		_queue_mutex.lock()
		_worker_error = "The recorder could not connect to FFmpeg's local frame stream."
		_worker_finished = true
		_queue_mutex.unlock()
		return
	while true:
		_frame_ready.wait()
		var frame := PackedByteArray()
		var should_finish := false
		_queue_mutex.lock()
		if not _frame_queue.is_empty():
			frame = _frame_queue.pop_front()
		should_finish = _worker_should_stop and _frame_queue.is_empty()
		_queue_mutex.unlock()
		if not frame.is_empty():
			var write_error := stream.put_data(frame)
			if write_error != OK:
				_queue_mutex.lock()
				_worker_error = "FFmpeg stopped accepting video frames."
				_queue_mutex.unlock()
				break
		if should_finish:
			break
	stream.disconnect_from_host()
	_queue_mutex.lock()
	_worker_finished = true
	_queue_mutex.unlock()


func _build_ffmpeg_args(codec: String, fps: int, output_path: String) -> PackedStringArray:
	var bitrate := maxi(1, int(ProjectSettings.get_setting("runtime_recorder/bitrate_mbps", 12)))
	var encoder := str(ProjectSettings.get_setting("runtime_recorder/encoder_h264", "libx264")) if codec == "h264" else str(ProjectSettings.get_setting("runtime_recorder/encoder_h265", "libx265"))
	var codec_tag := "avc1" if codec == "h264" else "hvc1"
	return PackedStringArray([
		"-hide_banner", "-loglevel", "error", "-y",
		"-f", "rawvideo", "-pixel_format", "rgba",
		"-video_size", "%dx%d" % [_frame_size.x, _frame_size.y],
		"-framerate", str(fps), "-i", "tcp://127.0.0.1:%d?listen=1" % _tcp_port,
		"-an", "-c:v", encoder, "-tag:v", codec_tag,
		"-b:v", "%dM" % bitrate, "-pix_fmt", "yuv420p",
		"-movflags", "+faststart", output_path
	])


func _finish_recording() -> void:
	is_finalizing = false
	_ffmpeg_pid = -1
	var exists := FileAccess.file_exists(last_output_path)
	var size := 0
	if exists:
		var output_file := FileAccess.open(last_output_path, FileAccess.READ)
		if output_file != null:
			size = output_file.get_length()
			output_file.close()
	if not exists or size == 0:
		_fail("FFmpeg did not produce a playable MP4.")
		return
	_show_indicator("Saved\n%s" % last_output_path.get_file(), Color(0.25, 0.9, 0.45))
	get_tree().create_timer(2.5, true, false, true).timeout.connect(_hide_indicator)
	print("Recording saved: ", last_output_path, " (dropped frames: ", dropped_frames, ")")
	recording_stopped.emit(last_output_path)


func _fail(message: String) -> void:
	is_recording = false
	is_finalizing = false
	_show_indicator("Recorder error\n%s" % message, Color(1.0, 0.28, 0.22))
	push_error(message)
	recording_failed.emit(message)


func _find_ffmpeg() -> String:
	var configured := str(ProjectSettings.get_setting("runtime_recorder/ffmpeg_path", ""))
	var candidates: Array[String] = [configured]
	# A Windows export can be distributed as a self-contained folder with
	# ffmpeg.exe beside the Godot executable. Keep this lookup ahead of PATH so
	# the bundled encoder is selected consistently on end-user machines.
	if OS.get_name() == "Windows":
		var executable_dir := OS.get_executable_path().get_base_dir()
		candidates.append(executable_dir.path_join("ffmpeg.exe"))
		for path_entry in OS.get_environment("PATH").split(";", false):
			candidates.append(path_entry.path_join("ffmpeg.exe"))
	else:
		candidates.append_array(["/opt/homebrew/bin/ffmpeg", "/usr/local/bin/ffmpeg", "/usr/bin/ffmpeg"])
	for candidate in candidates:
		if not candidate.is_empty() and FileAccess.file_exists(candidate):
			return candidate
	return ""


func _ffmpeg_install_hint() -> String:
	if OS.get_name() == "Windows":
		return "FFmpeg was not found. Put ffmpeg.exe beside the simulator EXE or add it to Windows PATH."
	return "FFmpeg was not found. Install it with: brew install ffmpeg"


func _globalize_directory(path: String) -> String:
	# The PCK inside an exported build is read-only. Redirect the default
	# res://recordings path to Godot's per-user writable data directory while
	# retaining res:// output during editor development.
	if path.begins_with("res://") and not OS.has_feature("editor"):
		path = "user://recordings"
	if path.begins_with("res://") or path.begins_with("user://"):
		return ProjectSettings.globalize_path(path)
	return path


func _create_indicator_window() -> void:
	_indicator_window = Window.new()
	_indicator_window.title = "MG400 Recorder"
	_indicator_window.size = Vector2i(360, 76)
	_indicator_window.borderless = true
	_indicator_window.always_on_top = true
	_indicator_window.unfocusable = true
	_indicator_window.visible = false
	add_child(_indicator_window)
	var background := ColorRect.new()
	background.color = Color(0.035, 0.04, 0.055, 0.94)
	background.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	_indicator_window.add_child(background)
	_indicator_label = Label.new()
	_indicator_label.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	_indicator_label.vertical_alignment = VERTICAL_ALIGNMENT_CENTER
	_indicator_label.add_theme_font_size_override("font_size", 17)
	_indicator_label.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	background.add_child(_indicator_label)
	var usable := DisplayServer.screen_get_usable_rect()
	_indicator_window.position = usable.position + usable.size - _indicator_window.size - Vector2i(24, 24)


func _show_indicator(text: String, color: Color) -> void:
	if _indicator_window == null:
		return
	_indicator_label.text = text
	_indicator_label.modulate = color
	_indicator_window.show()


func _hide_indicator() -> void:
	if _indicator_window != null and not is_recording and not is_finalizing:
		_indicator_window.hide()


func _update_recording_indicator() -> void:
	var elapsed := int((Time.get_ticks_usec() - _recording_started_usec) / 1000000)
	_indicator_label.text = "● REC  %02d:%02d\n%s to stop" % [elapsed / 60, elapsed % 60, HOTKEY_LABEL]
