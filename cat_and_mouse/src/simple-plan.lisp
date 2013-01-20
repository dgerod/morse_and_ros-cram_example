;;; ============================================================================
;;; simple-plan.lisp
;;; ============================================================================

(in-package :cmg)

(def-cram-function move-to-pos (goal &optional (distance-threshold 1.0))
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

(def-cram-function go-to (goal &optional (distance-threshold 1.0))
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

(def-cram-function follow-mouse-2 (&optional (distance-threshold 5.0))
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
           (loop do

             (format t "~a~%" (value *mouse-pose*) )
             (format t "~a~%" (value *cat-pose*) )
                                                     
             (format t "~a~%" (value (fl-funcall #'cl-transforms:v-dist  
                                    (fl-funcall
                                        #'cl-transforms:translation
                                        (fl-funcall
                                            #'pose-msg->transform
                                            *cat-pose*))
                                    (fl-funcall
                                        #'cl-transforms:translation
                                        (fl-funcall
                                            #'pose-msg->transform
                                            *mouse-pose*)) ))
             )
           )
           
           (wait-for reached-fl)
           (loop do
             (send-vel-cmd
               1.5
               (calculate-cat-angular-vel (cl-transforms:translation
                    (pose-msg->transform (value *mouse-pose*)))))
             (wait-duration 0.1)))
      (send-vel-cmd 0 0)))
)

(def-cram-function follow-mouse (&optional (distance-threshold 5.0))
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

