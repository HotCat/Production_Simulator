;;; mg400-traj-mode.el --- Edit MG400 Cartesian trajectory files -*- lexical-binding: t; -*-

;; Copyright (C) 2026
;; Author: Codex
;; Version: 0.1.0
;; Package-Requires: ((emacs "27.1"))
;; Keywords: tools, robotics, godot, trajectory

;;; Commentary:

;; `mg400-traj-mode' edits the line-oriented .traj files consumed by the
;; Godot MG400 simulator.  The format is intentionally easy to diff and edit:
;;
;;   [trajectory]
;;   feed_mm_s = 120
;;   acceleration_mm_s2 = 500
;;   junction_deviation_mm = 1
;;   loop = true
;;
;;   [waypoints]
;;   # X_mm  Y_mm  Z_mm  yaw_deg
;;   240 0 245 0
;;   335 0 245 0
;;
;; Coordinates use MG400/ROS millimetres (X, Y, Z, Z-up).  Yaw is optional and
;; is written in degrees.  The Godot planner converts these values to its
;; Y-up world coordinates and applies acceleration/junction lookahead.
;;
;; Key bindings:
;;
;;   C-c C-v   Validate this trajectory and report its waypoint count.
;;   C-c C-r   Run the Godot project with this .traj file loaded.
;;   C-c C-i   Insert a waypoint at the end of [waypoints].
;;   C-c C-s   Save the trajectory buffer.
;;
;; This follows the project’s `godot-pose-mode' conventions while keeping the
;; document format plain text rather than JSON.

;;; Code:

(require 'cl-lib)
(require 'conf-mode)
(require 'subr-x)

(defgroup mg400-traj nil
  "Edit MG400 Cartesian trajectory files."
  :group 'tools
  :prefix "mg400-traj-")

(defcustom mg400-traj-godot-executable
  (if (eq system-type 'darwin)
      "/Applications/Godot.app/Contents/MacOS/Godot"
    "godot")
  "Godot executable used by `mg400-traj-run'."
  :type 'file
  :group 'mg400-traj)

(defcustom mg400-traj-default-project-root nil
  "Optional absolute project root used when a .traj file is outside the project.
When nil, `mg400-traj-run' searches upward for project.godot."
  :type '(choice (const :tag "Search from the trajectory file" nil)
                 directory)
  :group 'mg400-traj)

(defvar mg400-traj--run-process nil
  "The Godot process most recently started by `mg400-traj-run'.")

(defconst mg400-traj--number-regexp
  "[-+]?[0-9]+\\(?:\\.[0-9]*\\)?\\(?:[eE][-+]?[0-9]+\\)?\\|[-+]?\\.[0-9]+\\(?:[eE][-+]?[0-9]+\\)?")

(defconst mg400-traj--font-lock-keywords
  `(("^\\[[[:alpha:]_]+\\]" . font-lock-function-name-face)
    ("^[[:space:]]*\\([[:alpha:]_][[:alnum:]_]*\\)[[:space:]]*="
     (1 font-lock-variable-name-face))
    (,(concat "\\_<" mg400-traj--number-regexp "\\_>")
     . font-lock-constant-face)
    ("\\_<\\(?:true\\|false\\|yes\\|no\\|mm\\|mg400-mm\\)\\_>"
     . font-lock-builtin-face)
    ("^\\s-*#.*$" . font-lock-comment-face))
  "Font-lock rules for MG400 .traj files.")

(defvar mg400-traj-mode-map
  (let ((map (make-sparse-keymap)))
    (define-key map (kbd "C-c C-v") #'mg400-traj-validate)
    (define-key map (kbd "C-c C-r") #'mg400-traj-run)
    (define-key map (kbd "C-c C-i") #'mg400-traj-insert-waypoint)
    (define-key map (kbd "C-c C-s") #'save-buffer)
    map)
  "Keymap for `mg400-traj-mode'.")

(defun mg400-traj--error (line message)
  "Signal a user error at LINE with MESSAGE."
  (user-error "%s:%d: %s"
              (or (and buffer-file-name (file-name-nondirectory buffer-file-name))
                  "trajectory")
              line
              message))

(defun mg400-traj--number (token line)
  "Parse numeric TOKEN from LINE or signal a useful user error."
  (unless (string-match-p (concat "\\`" mg400-traj--number-regexp "\\'") token)
    (mg400-traj--error line (format "expected a number, got %S" token)))
  (string-to-number token))

(defun mg400-traj--boolean (token line)
  "Parse boolean TOKEN from LINE or signal a useful user error."
  (let ((normalized (downcase token)))
    (cond
     ((member normalized '("true" "yes" "1")) t)
     ((member normalized '("false" "no" "0")) nil)
     (t (mg400-traj--error line "loop must be true/false")))))

(defun mg400-traj--parse-buffer ()
  "Parse the current buffer and return a property list.
Signal `user-error' with the source line on malformed input."
  (let ((section nil)
        (feed 120.0)
        (acceleration 500.0)
        (junction 1.0)
        (loop t)
        (waypoints nil)
        (line-number 0))
    (dolist (raw-line (split-string (buffer-string) "\n"))
      (setq line-number (1+ line-number))
      (let* ((comment-free (car (split-string raw-line "#" nil)))
             (line (string-trim (or comment-free ""))))
        (unless (string-empty-p line)
          (cond
           ((string-match "\\`\\[\\([^]]+\\)\\]\\'" line)
            (setq section (downcase (match-string 1 line)))
            (unless (member section '("trajectory" "waypoints"))
              (mg400-traj--error line-number
                                  (format "unknown section [%s]" section))))
           ((and (equal section "trajectory")
                 (string-match "\\`\\([^=]+\\)=\\(.*\\)\\'" line))
            (let ((key (downcase (string-trim (match-string 1 line))))
                  (value (string-trim (match-string 2 line))))
              (cond
               ((member key '("feed_mm_s" "velocity_mm_s" "speed_mm_s"))
                (setq feed (mg400-traj--number value line-number)))
               ((member key '("acceleration_mm_s2" "accel_mm_s2"))
                (setq acceleration (mg400-traj--number value line-number)))
               ((member key '("junction_deviation_mm" "junction_mm"))
                (setq junction (mg400-traj--number value line-number)))
               ((member key '("loop" "loop_trajectory"))
                (setq loop (mg400-traj--boolean value line-number)))
               ((equal key "units")
                (unless (member (downcase value) '("mm" "mg400-mm" "mg400"))
                  (mg400-traj--error line-number "units must be mm or mg400-mm")))
               (t
                (mg400-traj--error line-number
                                    (format "unknown parameter %s" key))))))
           ((equal section "waypoints")
            (let* ((payload (if (string-match
                                "\\`waypoint[[:space:]]*=[[:space:]]*\\(.*\\)\\'"
                                line)
                                (match-string 1 line)
                              line))
                   (fields (split-string payload "[[:space:]]+" t)))
              (unless (memq (length fields) '(3 4))
                (mg400-traj--error line-number
                                    "waypoint needs X Y Z and optional yaw degrees"))
              (let ((values (mapcar (lambda (field)
                                     (mg400-traj--number field line-number))
                                   fields)))
                (push (list (nth 0 values) (nth 1 values) (nth 2 values)
                            (or (nth 3 values) 0.0))
                      waypoints))))
           (t
            (mg400-traj--error line-number
                                "expected a section, parameter, or waypoint"))))))
    (setq waypoints (nreverse waypoints))
    (unless (> (length waypoints) 1)
      (user-error "Trajectory needs at least two waypoints"))
    (when (<= feed 0.0)
      (user-error "feed_mm_s must be greater than zero"))
    (when (<= acceleration 0.0)
      (user-error "acceleration_mm_s2 must be greater than zero"))
    (when (< junction 0.0)
      (user-error "junction_deviation_mm must be zero or positive"))
    (list :feed-mm-s feed
          :acceleration-mm-s2 acceleration
          :junction-deviation-mm junction
          :loop loop
          :waypoints waypoints)))

(defun mg400-traj-validate ()
  "Validate the current .traj buffer and report its motion parameters."
  (interactive)
  (let ((document (mg400-traj--parse-buffer)))
    (message "Valid MG400 trajectory: %d waypoints, %.1f mm/s, %.1f mm/s², loop %s"
             (length (plist-get document :waypoints))
             (plist-get document :feed-mm-s)
             (plist-get document :acceleration-mm-s2)
             (if (plist-get document :loop) "on" "off"))))

(defun mg400-traj--waypoint-section-end ()
  "Return the buffer position where a new waypoint should be inserted."
  (save-excursion
    (goto-char (point-min))
    (unless (re-search-forward "^\\[waypoints\\][[:space:]]*$" nil t)
      (user-error "Cannot find [waypoints] section"))
    (forward-line 1)
    (while (and (not (eobp))
                (not (looking-at "^\\[[[:alpha:]_]+\\][[:space:]]*$")))
      (forward-line 1))
    (point)))

(defun mg400-traj-insert-waypoint (x y z &optional yaw)
  "Insert a waypoint with MG400 millimetre X, Y, Z and optional YAW degrees."
  (interactive
   (list (read-number "MG400 X (mm): ")
         (read-number "MG400 Y (mm): ")
         (read-number "MG400 Z (mm): ")
         (read-number "Yaw (degrees, default 0): " 0.0)))
  (barf-if-buffer-read-only)
  (mg400-traj--parse-buffer)
  (let ((position (mg400-traj--waypoint-section-end)))
    (goto-char position)
    (unless (bolp) (insert "\n"))
    (insert (format "%.3f %.3f %.3f %.3f\n" x y z (or yaw 0.0)))
    (message "Inserted waypoint %.3f %.3f %.3f %.3f" x y z (or yaw 0.0))))

(defun mg400-traj--project-root ()
  "Find the Godot project root for the current trajectory buffer."
  (let ((located (locate-dominating-file (or buffer-file-name default-directory)
                                         "project.godot")))
    (or (and mg400-traj-default-project-root
             (file-directory-p mg400-traj-default-project-root)
             (file-name-as-directory
              (expand-file-name mg400-traj-default-project-root)))
        (and located
             (file-name-as-directory (expand-file-name located)))
        (user-error "Cannot locate project.godot; customize mg400-traj-default-project-root"))))

(defun mg400-traj-run ()
  "Run the Godot project with the current .traj document loaded."
  (interactive)
  (unless buffer-file-name
    (user-error "Save the .traj buffer before running it"))
  (mg400-traj--parse-buffer)
  (save-buffer)
  (when (process-live-p mg400-traj--run-process)
    (user-error "A MG400 trajectory preview is already running"))
  (let* ((project-root (mg400-traj--project-root))
         (trajectory-file (file-truename buffer-file-name))
         (log-buffer (get-buffer-create "*MG400 Trajectory Preview*")))
    (setq mg400-traj--run-process
          (start-process "mg400-traj-preview" log-buffer
                         mg400-traj-godot-executable
                         "--path" project-root
                         "--"
                         (concat "--trajectory=" trajectory-file)))
    (set-process-query-on-exit-flag mg400-traj--run-process nil)
    (set-process-sentinel
     mg400-traj--run-process
     (lambda (process event)
       (when (and (not (process-live-p process))
                  (not (zerop (process-exit-status process))))
         (display-buffer (process-buffer process))
         (message "MG400 trajectory preview failed: %s"
                  (string-trim event)))))
    (message "Started MG400 trajectory preview: %s" trajectory-file)))

;;;###autoload
(define-derived-mode mg400-traj-mode conf-mode "MG400-Traj"
  "Major mode for editable MG400 Cartesian .traj documents.

Use `C-c C-v' to validate, `C-c C-i' to insert a waypoint, and `C-c C-r'
to launch the Godot project with this trajectory loaded."
  (setq-local comment-start "#")
  (setq-local comment-end "")
  (setq-local font-lock-defaults '(mg400-traj--font-lock-keywords))
  (setq-local indent-tabs-mode nil))

;;;###autoload
(add-to-list 'auto-mode-alist '("\\.traj\\'" . mg400-traj-mode))

(provide 'mg400-traj-mode)

;;; mg400-traj-mode.el ends here
