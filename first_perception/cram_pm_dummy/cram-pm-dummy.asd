(defsystem cram-pm-dummy
  :depends-on (roslisp cram-language process-modules simple_perception-msg turtlesim-msg cl-transforms)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "process-module" :depends-on ("package"))
             (:file "first-plan"     :depends-on ("package"))
             (:file "node-base"      :depends-on ("package"))))))
