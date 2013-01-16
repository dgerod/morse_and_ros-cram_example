(defsystem cat-and-mouse
  :depends-on (roslisp cram-language process-modules geometry_msgs-msg nav_msgs-msg turtlesim-msg cl-transforms)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "simple-plan"    :depends-on ("package"))
             (:file "node-base"      :depends-on ("package"))))))
