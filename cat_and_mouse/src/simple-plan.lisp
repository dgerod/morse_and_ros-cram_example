;;; ============================================================================
;;; simple-plan.lisp
;;; ============================================================================

(in-package :cmg)

(def-cram-function move-to (goal-pos &optional (distance-threshold 1.0))
  (let ((reached-fl (< (fl-funcall #'cl-transforms:v-dist
                                   (fl-funcall
                                       #'cl-transforms:translation
                                       (fl-funcall
                                           #'pose-msg->transform
                                           *cat-pose-2*))
                                   goal-pos)
                        distance-threshold)))
    (unwind-protect
         (pursue
           (wait-for reached-fl)
           (loop do
             (send-vel-cmd
               1.5
               (calculate-cat-angular-vel goal-pos))
             (wait-duration 0.1)))
      (send-vel-cmd 0 0)))
)

(def-cram-function go-to (goal &optional (distance-threshold 1.0))
  (let ((reached-fl (< (fl-funcall #'cl-transforms:v-dist
                                   (fl-funcall
                                       #'cl-transforms:translation
                                       (fl-funcall
                                           #'pose-msg->transform
                                           *cat-pose-2*))
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

(def-cram-function go-to-2 (goal &optional (distance-threshold 1.0))
  ;;(let ((reached-fl (< (fl-funcall #'cl-transforms:v-dist
  ;;                               (fl-funcall
  ;;                                #'cl-transforms:translation
  ;;                                (fl-funcall
  ;;                                 #'pose-msg->transform
  ;;                                 *cat-pose-2*))
  ;;                               goal)
  ;;                   distance-threshold)))
    (unwind-protect
         (pursue
           ;;(wait-for reached-fl)
           (loop do
             (send-vel-cmd
               1.5
               (calculate-cat-angular-vel (value goal)))
             (wait-duration 0.1));;)
      (send-vel-cmd 0 0)))
)

(def-cram-function follow-mouse (&optional (distance-threshold 5.0))
  (let ((reached-fl (< (fl-funcall #'cl-transforms:v-dist  
                                    (fl-funcall
                                        #'cl-transforms:translation
                                        (fl-funcall
                                            #'pose-msg->transform
                                            *cat-pose-2*))
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
             (wait-duration 0.1)))
      (send-vel-cmd 0 0)))
)

(def-cram-function follow-mouse-2 (&optional (distance-threshold 5.0))
  (let ((reached-fl (< (fl-funcall #'cl-transforms:v-dist  
                                    (fl-funcall
                                        #'cl-transforms:translation
                                        (fl-funcall
                                            #'pose-msg->transform
                                            *cat-pose-2*))
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

