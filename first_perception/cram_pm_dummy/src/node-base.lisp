;;; ============================================================================
;;; node-base.lisp
;;; ============================================================================

(in-package :c-pm-d)

;;;  ---------------------------------------------------------------------------

(defvar *object-id* (make-fluent :name :object-id) "current object detected")
(defvar *object-list* (list) "list of objects detected")

(defvar *object-sub* nil "object subscription client")

;;;  ---------------------------------------------------------------------------

(defun start-node ()
  (roslisp:start-ros-node "my-cram-client"))

(defun start-node-with-name (name)
  (roslisp:start-ros-node (format nil "~a" name)))

(defun stop-node ()
  (roslisp:shutdown-ros-node))

(defun init-ros-turtle ()
  "subscribes to topics for a turtle and binds callbacks. `name' specifies the name of the turtle."
  (setf *object-sub* (subscribe "simple_perception/percept"
                                "simple_perception/Percept"
                                #'object-cb)))

(defun object-cb (msg)
  "Callback for object ids"
  (setf (value *object-id*) msg)
  (ros-info my-cram-client "Object received")
  (setf *object-list* (append *object-list* (list msg)))
 )

;;;  ---------------------------------------------------------------------------

(defun pose-msg->transform (msg)
  "returns a transform proxy that allows to transform into the frame given by x, y, and theta of msg."
  (with-fields (x y theta) msg
    (cl-transforms:make-transform
     (cl-transforms:make-3d-vector x y 0)
     (cl-transforms:axis-angle->quaternion
      (cl-transforms:make-3d-vector 0 0 1)
      theta))))

(defun relative-angle-to (goal pose)
  "Given a pose as 3d-pose msg and a goal as 3d vector, calculate the angle by which the pose has to be turned to point toward the goal."
  (let ((diff-pose (cl-transforms:transform-point
                     (cl-transforms:transform-inv
                       (pose-msg->transform pose))
                     goal)))
    (atan
      (cl-transforms:y diff-pose)
      (cl-transforms:x diff-pose))))

(defun calculate-angular-cmd (goal &optional (ang-vel-factor 4))
  "Uses the current turtle pose and calculates the angular velocity
  command to turn towards the goal."
  (* ang-vel-factor
     (relative-angle-to goal (value *turtle-pose*))))

(def-cram-function move-to (goal &optional (distance-threshold 0.1))
  (let ((reached-fl (< (fl-funcall #'cl-transforms:v-dist
                                   (fl-funcall
                                    #'cl-transforms:translation
                                    (fl-funcall
                                     #'pose-msg->transform
                                     *turtle-pose*))
                                   goal)
                       distance-threshold)))
    (unwind-protect
         (pursue
           (wait-for reached-fl)
           (loop do
             (send-vel-cmd
               1.5
               (calculate-angular-cmd goal))
             (wait-duration 0.1)))
      (send-vel-cmd 0 0))))

;;;  ===========================================================================
