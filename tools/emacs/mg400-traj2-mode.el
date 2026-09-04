;;; mg400-traj2-mode.el --- Edit and upload FR3 .traj2 programs -*- lexical-binding: t; -*-

;; A small conf-mode based editor for the Fairino3 simulator's six-axis
;; Cartesian format. Each waypoint has XYZ/Pitch/Roll/Yaw plus J1-J6 (degrees).
;; C-c C-v validates locally, C-c C-r starts Godot, and
;; C-c C-u uploads the saved file to an already-running Fairino3 scene. C-c C-p
;; captures the current live robot pose and appends all 12 fields. C-c C-g
;; sends exactly 12 numeric fields in the active region as an immediate goto
;; command to the running simulator.

(require 'conf-mode)
(require 'subr-x)
(require 'seq)

(defgroup mg400-traj2 nil "Edit FR3 .traj2 trajectory files." :group 'tools)
(defcustom mg400-traj2-godot-executable
  (if (eq system-type 'darwin) "/Applications/Godot.app/Contents/MacOS/Godot" "godot")
  "Godot executable used to launch the simulator." :type 'file :group 'mg400-traj2)
(defcustom mg400-traj2-runtime-command ".runtime/trajectory2.command"
  "Project-relative command file used for runtime upload." :type 'string :group 'mg400-traj2)
(defcustom mg400-traj2-runtime-goto ".runtime/trajectory2.goto"
  "Project-relative command file used for a live 12-field goto pose." :type 'string :group 'mg400-traj2)
(defvar mg400-traj2--process nil)
(defconst mg400-traj2--number
  "[-+]?[0-9]+\\(?:\\.[0-9]*\\)?\\(?:[eE][-+]?[0-9]+\\)?\\|[-+]?\\.[0-9]+\\(?:[eE][-+]?[0-9]+\\)?")
(defconst mg400-traj2--font-lock
  `(("^\\[[[:alpha:]_]+\\]" . font-lock-function-name-face)
    ("^[[:space:]]*\\([[:alpha:]_][[:alnum:]_]*\\)[[:space:]]*=" (1 font-lock-variable-name-face))
    (,(concat "\\_<" mg400-traj2--number "\\_>") . font-lock-constant-face)
    ("\\_<\\(?:true\\|false\\|yes\\|no\\|fr3-mm\\)\\_>" . font-lock-builtin-face)
    ("^[[:space:]]*\\_<\\(?:delay\\|trigger\\|thin\\|long\\|return\\)\\_>" . font-lock-keyword-face)
    ("^\\s-*#.*$" . font-lock-comment-face)))

(defun mg400-traj2--error (line message)
  (user-error "%s:%d: %s" (or (and buffer-file-name (file-name-nondirectory buffer-file-name)) "trajectory2") line message))
(defun mg400-traj2--number (token line)
  (unless (string-match-p (concat "\\`" mg400-traj2--number "\\'") token)
    (mg400-traj2--error line (format "expected a number, got %S" token)))
  (string-to-number token))
(defun mg400-traj2--bool (token line)
  (let ((v (downcase token)))
    (cond ((member v '("true" "yes" "1")) t)
          ((member v '("false" "no" "0")) nil)
          (t (mg400-traj2--error line "value must be true/false")))))

(defun mg400-traj2--parse-buffer ()
  (let ((section nil) (feed 120.0) (acceleration 500.0) (junction 1.0)
        (loop t) (waypoints 0) (events 0) (line-number 0))
    (dolist (raw (split-string (buffer-string) "\n"))
      (setq line-number (1+ line-number))
      (let ((line (string-trim (car (split-string raw "#" nil)))))
        (unless (string-empty-p line)
          (cond
           ((and (string-prefix-p "[" line) (string-suffix-p "]" line))
            (setq section (downcase (string-trim (substring line 1 -1))))
            (unless (member section '("trajectory" "overlays" "waypoints"))
              (mg400-traj2--error line-number "unknown section")))
           ((and (member section '("trajectory" "overlays"))
                 (string-match "\\`\\([^=]+\\)=\\(.*\\)\\'" line))
            (let ((key (downcase (string-trim (match-string 1 line))))
                  (value (string-trim (match-string 2 line))))
              (if (equal section "overlays")
                  (progn
                    (unless (member key '("status" "joints" "pose" "orientation" "tcp_offset" "translation" "pickup" "recording" "pendant"))
                      (mg400-traj2--error line-number "unknown overlay"))
                    (mg400-traj2--bool value line-number))
                (cond
                 ((equal key "units") (unless (equal (downcase value) "fr3-mm") (mg400-traj2--error line-number "units must be fr3-mm")))
                 ((equal key "feed_mm_s") (setq feed (mg400-traj2--number value line-number)))
                 ((equal key "acceleration_mm_s2") (setq acceleration (mg400-traj2--number value line-number)))
                 ((equal key "junction_deviation_mm") (setq junction (mg400-traj2--number value line-number)))
                 ((equal key "loop") (setq loop (mg400-traj2--bool value line-number)))
                 (t (mg400-traj2--error line-number (format "unknown parameter %s" key)))))))
           ((equal section "waypoints")
            (let* ((fields (split-string (replace-regexp-in-string "=" " " line) "[[:space:]]+" t))
                   (command (downcase (or (car fields) ""))))
              (cond
               ((member command '("delay" "trigger"))
                (unless (and (> waypoints 0) (= (length fields) 2))
                  (mg400-traj2--error line-number (format "%s needs one argument after a waypoint" command)))
                (when (equal command "delay") (when (< (mg400-traj2--number (nth 1 fields) line-number) 0) (mg400-traj2--error line-number "delay must be positive")))
                (setq events (1+ events)))
               ((and (= (length fields) 2)
                     (equal command "thin") (equal (downcase (nth 1 fields)) "side"))
                (unless (> waypoints 0) (mg400-traj2--error line-number "thin side needs a preceding waypoint"))
                (setq events (1+ events)))
               ((and (= (length fields) 2)
                     (equal command "long") (equal (downcase (nth 1 fields)) "side"))
                (unless (> waypoints 0) (mg400-traj2--error line-number "long side needs a preceding waypoint"))
                (setq events (1+ events)))
               ((and (= (length fields) 2)
                     (equal command "return") (equal (downcase (nth 1 fields)) "product"))
                (unless (> waypoints 0) (mg400-traj2--error line-number "return product needs a preceding waypoint"))
                (setq events (1+ events)))
               (t
                (unless (= (length fields) 12) (mg400-traj2--error line-number "waypoint needs exactly X Y Z Pitch Roll Yaw J1 J2 J3 J4 J5 J6"))
                (mapc (lambda (x) (mg400-traj2--number x line-number)) fields)
                (setq waypoints (1+ waypoints))))))
           (t (mg400-traj2--error line-number "expected a section, assignment, or waypoint"))))))
    (unless (> waypoints 1) (user-error "Trajectory2 needs at least two waypoints"))
    (when (<= feed 0) (user-error "feed_mm_s must be positive"))
    (when (<= acceleration 0) (user-error "acceleration_mm_s2 must be positive"))
    (when (< junction 0) (user-error "junction_deviation_mm must be non-negative"))
    (list :feed feed :acceleration acceleration :junction junction :loop loop :waypoints waypoints :events events)))

(defun mg400-traj2--project-root ()
  (or (locate-dominating-file (or buffer-file-name default-directory) "project.godot")
      (user-error "Cannot locate project.godot")))
(defun mg400-traj2-validate ()
  (interactive)
  (let ((d (mg400-traj2--parse-buffer)))
    (message "Valid FR3 trajectory2: %d waypoints, %.1f mm/s, loop %s" (plist-get d :waypoints) (plist-get d :feed) (if (plist-get d :loop) "on" "off"))))
(defun mg400-traj2-run ()
  (interactive)
  (unless buffer-file-name (user-error "Save the .traj2 buffer before running"))
  (mg400-traj2--parse-buffer) (save-buffer)
  (when (process-live-p mg400-traj2--process) (user-error "A Fairino3 preview is already running"))
  (let ((root (file-name-as-directory (expand-file-name (mg400-traj2--project-root))))
        (file (file-truename buffer-file-name)))
    (setq mg400-traj2--process (start-process "mg400-traj2-preview" "*MG400 Trajectory2 Preview*"
                                              mg400-traj2-godot-executable "--path" root "--" (concat "--trajectory2=" file)))
    (set-process-query-on-exit-flag mg400-traj2--process nil)
    (message "Started Fairino3 trajectory2: %s" file)))
(defun mg400-traj2-upload ()
  (interactive)
  (unless buffer-file-name (user-error "Save the .traj2 buffer before uploading"))
  (mg400-traj2--parse-buffer) (save-buffer)
  (let* ((trajectory-file (file-truename buffer-file-name))
         (root (file-name-as-directory (expand-file-name (mg400-traj2--project-root))))
         (command (expand-file-name mg400-traj2-runtime-command root)))
    (make-directory (file-name-directory command) t)
    ;; The second line makes repeated uploads of the same path observable even
    ;; on filesystems whose modification time has one-second resolution.
    (with-temp-file command
      (insert trajectory-file "\n# upload " (format "%.6f" (float-time)) "\n"))
    (message "Uploaded trajectory2 to running simulator: %s" buffer-file-name)))

(defun mg400-traj2-goto-region (beg end)
  "Send the active region's 12-field FR3 pose to the running simulator.
The region must contain exactly X Y Z Pitch Roll Yaw J1 J2 J3 J4 J5 J6,
all in the same millimetre/degree units as a .traj2 waypoint."
  (interactive "r")
  (unless (use-region-p) (user-error "Mark one 12-field waypoint first"))
  (let* ((tokens (split-string (string-trim (buffer-substring-no-properties beg end))
                               "[[:space:]]+" t)))
    (unless (= (length tokens) 12)
      (user-error "Goto region needs exactly 12 numeric fields (got %d)" (length tokens)))
    (mapc (lambda (token) (mg400-traj2--number token (line-number-at-pos beg))) tokens)
    (let* ((root (file-name-as-directory (expand-file-name (mg400-traj2--project-root))))
           (command (expand-file-name mg400-traj2-runtime-goto root)))
      (make-directory (file-name-directory command) t)
      (with-temp-file command
        (insert (mapconcat #'identity tokens " ") "\n# goto " (format "%.6f" (float-time)) "\n"))
      (deactivate-mark)
      (message "Sent FR3 goto pose to running simulator"))))

(defun mg400-traj2--waypoint-section-end ()
  "Return the insertion position at the end of the [waypoints] section."
  (save-excursion
    (goto-char (point-min))
    (unless (re-search-forward "^\\[waypoints\\][[:space:]]*$" nil t)
      (user-error "Cannot find [waypoints] section"))
    (forward-line 1)
    (while (and (not (eobp)) (not (looking-at "^\\[[[:alpha:]_]+\\][[:space:]]*$")))
      (forward-line 1))
    (point)))

(defun mg400-traj2-insert-waypoint (x y z pitch roll yaw j1 j2 j3 j4 j5 j6)
  (interactive (list (read-number "X (mm): ") (read-number "Y (mm): ") (read-number "Z (mm): ")
                     (read-number "Pitch (deg): ") (read-number "Roll (deg): ") (read-number "Yaw (deg): ")
                     (read-number "J1 (deg): ") (read-number "J2 (deg): ") (read-number "J3 (deg): ")
                     (read-number "J4 (deg): ") (read-number "J5 (deg): ") (read-number "J6 (deg): ")))
  (barf-if-buffer-read-only) (mg400-traj2--parse-buffer)
  (goto-char (mg400-traj2--waypoint-section-end))
  (unless (bolp) (insert "\n"))
  (insert (format "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n"
                 x y z pitch roll yaw j1 j2 j3 j4 j5 j6)))

(defun mg400-traj2-capture-pose ()
  "Append the latest 12-field pose published by the running Fairino3 scene."
  (interactive)
  (barf-if-buffer-read-only)
  (mg400-traj2--parse-buffer)
  (let* ((root (file-name-as-directory (expand-file-name (mg400-traj2--project-root))))
         (pose-file (expand-file-name ".runtime/trajectory2.pose" root)))
    (unless (file-readable-p pose-file)
      (user-error "No live pose found; start fairino3_demo.tscn first"))
    (let ((values
           (with-temp-buffer
             (insert-file-contents pose-file)
             (let* ((lines (split-string (buffer-string) "\n" t))
                    (data-line (seq-find (lambda (candidate)
                                           (not (string-match-p "^[[:space:]]*#" candidate)))
                                         lines))
                    (fields (and data-line (split-string (string-trim data-line) "[[:space:]]+" t))))
               (unless data-line (user-error "Live pose file is empty"))
               (unless (= (length fields) 12)
                 (user-error "Live pose file is incomplete (expected 12 values)"))
               (mapcar #'string-to-number fields)))))
      (goto-char (mg400-traj2--waypoint-section-end))
      (unless (bolp) (insert "\n"))
      (insert (apply #'format "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n" values))
      (message "Captured current FR3 pose as waypoint"))))

(defvar mg400-traj2-mode-map
  (let ((map (make-sparse-keymap)))
    (define-key map (kbd "C-c C-v") #'mg400-traj2-validate)
    (define-key map (kbd "C-c C-r") #'mg400-traj2-run)
    (define-key map (kbd "C-c C-u") #'mg400-traj2-upload)
    (define-key map (kbd "C-c C-g") #'mg400-traj2-goto-region)
    (define-key map (kbd "C-c C-p") #'mg400-traj2-capture-pose)
    (define-key map (kbd "C-c C-i") #'mg400-traj2-insert-waypoint)
    (define-key map (kbd "C-c C-s") #'save-buffer) map))

;;;###autoload
(define-derived-mode mg400-traj2-mode conf-mode "MG400-Traj2"
  "Major mode for FR3 .traj2 Cartesian trajectory programs."
  (setq-local comment-start "#" comment-end "" font-lock-defaults '(mg400-traj2--font-lock) indent-tabs-mode nil))
;;;###autoload
(add-to-list 'auto-mode-alist '("\\.traj2\\'" . mg400-traj2-mode))
(provide 'mg400-traj2-mode)
;;; mg400-traj2-mode.el ends here
