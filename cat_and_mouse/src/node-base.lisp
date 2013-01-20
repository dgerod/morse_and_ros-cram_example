;;; ============================================================================
;;; node-base.lisp
;;; ============================================================================

(in-package :cmg)

;;; ---------------------------------------------------------------------------

(defvar *mouse-pose* (make-fluent :name :mouse-pose) "current pose of mouse")
(defvar *mouse-pose-sub* nil "pose subscription client")

(defvar *cat-pose* (make-fluent :name :cat-pose) "current pose of cat")
(defvar *cat-pose-sub* nil "pose subscription client")
(defvar *cat-cmd-vel-pub* nil "command velocity subscription client")

(defvar *dbg-mouse-pose* (make-fluent :name :dbg-mouse-pose) "current pose of cat")

(defvar *mouse-pose-pub* nil "command velocity subscription client")
(defvar *cat-pose-pub* nil "command velocity subscription client")

;;; ---------------------------------------------------------------------------

(defun start-node ()
  (roslisp:start-ros-node "cat-client"))

(defun start-node-with-name (name)
  (roslisp:start-ros-node (format nil "~a" name)))

(defun stop-node ()
  (roslisp:shutdown-ros-node))

(defun init-ros-cat ()
  "subscribes to topics for a cat and binds callbacks. `name' specifies the name of the turtle."
  (setf *mouse-pose-sub* (subscribe "MOUSE/Pose"
                          "nav_msgs/Odometry"
                          #'mouse-pose-cb))  
  (setf *cat-pose-sub* (subscribe "CAT/Pose.001"
                        "nav_msgs/Odometry"
                        #'cat-pose-cb))  
  (setf *cat-cmd-vel-pub* (advertise "CAT/Motion_Controller.001"
                           "geometry_msgs/Twist"))
    
  ;;(setf *mouse-pose-pub* (advertise "MOUSE/TurtlePose"
  ;;                         "geometry_msgs/Twist"))
  ;;(setf *cat-pose-pub* (advertise "CAT/TurtlePose"
  ;;                         "geometry_msgs/Twist"))
)

(defun mouse-pose-cb (msg)
  "Callback for pose values"
  (setf (value *mouse-pose*) 
    (turlesim-msg-from-nav-msg msg))
  ;;
  (setf (value *dbg-mouse-pose*) msg)
  ;;(publish *mouse-pose-pub* (turlesim-msg-from-nav-msg msg))
)

(defun cat-pose-cb (msg)
  "Callback for cat pose values"
  (setf (value *cat-pose*) 
    (turlesim-msg-from-nav-msg msg))
  ;;
  ;;(publish *cat-pose-pub* (value *cat-pose*))
)

(defun create-vel-cmd (lin ang) 
  "function to send velocity commands"
  (roslisp:make-message "geometry_msgs/Twist" 
    :linear  (make-message "geometry_msgs/vector3" (x) lin)
    :angular (make-message "geometry_msgs/vector3" (z) ang))
)

(defun send-vel-cmd (lin ang)
   "function to send velocity commands"
   (setf msg (create-vel-cmd lin ang))
   (publish *cat-cmd-vel-pub* msg)
)

;;; ---------------------------------------------------------------------------

(defun parse-nav-message (msg) 
  (roslisp:with-fields (pose) msg
    (setf tmp1 pose) )
  (roslisp:with-fields (pose) tmp1
    (setf tmp2 pose) )
  (roslisp:with-fields (position orientation) tmp2
    (setf ori orientation)
    (setf pos position) )
  (list pos ori)
)

(defun create-turlesim-msg (x y rot) 
  (make-message "turtlesim/Pose" :x x :y y :theta rot)
)

(defun turlesim-msg-from-nav-msg (msg) 
  (setf pose (parse-nav-message msg))
  (roslisp:with-fields (X Y Z) (first pose)
    (setf x1 X)
    (setf y1 Y))  
  (roslisp:with-fields (X Y Z W) (second pose)
    (setf q1 (make-quaternion X Y Z W))
    (setf theta (cl-transforms:get-yaw q1)))  
  (create-turlesim-msg x1 y1 theta) 
)

;;; ---------------------------------------------------------------------------

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
      (cl-transforms:x diff-pose)))
)

(defun calculate-cat-angular-vel (goal &optional (ang-vel-factor 2.0))
  "Uses the current turtle pose and calculates the angular velocity
  command to turn towards the goal."
  (* ang-vel-factor
     (relative-angle-to goal (value *cat-pose*)))
)

;;; ===========================================================================
