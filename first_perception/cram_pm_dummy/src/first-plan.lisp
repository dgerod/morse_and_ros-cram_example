(in-package :c-pm-d)

(def-cram-function first-plan (goal &optional (distance-threshold 0.1))
  "My first CRAM plan."
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
