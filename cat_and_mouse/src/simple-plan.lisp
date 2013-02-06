;;; ============================================================================
;;; simple-plan.lisp
;;; ============================================================================

(in-package :cmg)

(def-cram-function move-to-pos (goal &optional (distance-threshold 1.0))
  "Plan to move cat to a fixed position defined as 3d-vector"
  (let ((reached-fl (< (fl-funcall #'cl-transforms:v-dist
                                   (fl-funcall
                                       #'cl-transforms:translation
                                       (fl-funcall
                                           #'pose-msg->transform
                                           *cat-pose*))
                                   goal)
                        distance-threshold)))
    (unwind-protect
         (pursue
           (wait-for reached-fl)
           (loop do
             (send-vel-cmd
               1.5
               (calculate-cat-angular-vel goal))
             (wait-duration 0.1)))
      (send-vel-cmd 0 0)))
)

;;; ---------------------------------------------------------------------------

(def-cram-function go-to (goal &optional (distance-threshold 2.0))
  "Plan to follow a goal defined as a fluent"
  (let ((reached-fl (< (fl-funcall #'cl-transforms:v-dist
                                   (fl-funcall
                                       #'cl-transforms:translation
                                       (fl-funcall
                                           #'pose-msg->transform
                                           *cat-pose*))
                                   goal)
                        distance-threshold)))
    (unwind-protect
         (pursue
           (wait-for reached-fl)
           (loop do
             (send-vel-cmd
               1.5
               (calculate-cat-angular-vel (value goal)))
             (wait-duration 0.1)))
      (send-vel-cmd 0 0)))
)

(def-cram-function follow-mouse (&optional (distance-threshold 2.0))
  "Plan to follow the mouse"
  (let ((reached-fl (< (fl-funcall #'cl-transforms:v-dist  
                                    (fl-funcall
                                        #'cl-transforms:translation
                                        (fl-funcall
                                            #'pose-msg->transform
                                            *cat-pose*))
                                    (fl-funcall
                                        #'cl-transforms:translation
                                        (fl-funcall
                                            #'pose-msg->transform
                                            *mouse-pose*)) )                                            
                       distance-threshold)))
    (unwind-protect
         (pursue
           (wait-for reached-fl)           
           (loop do
             (send-vel-cmd
               1.5
               (calculate-cat-angular-vel (cl-transforms:translation
                    (pose-msg->transform (value *mouse-pose*)))))
             (wait-duration 0.1))
         )
      (send-vel-cmd 0 0)
    ))
)

;;; ============================================================================

