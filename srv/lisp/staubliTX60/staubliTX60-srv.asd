
(in-package :asdf)

(defsystem "staubli_tx60-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils)
  :components ((:file "_package")
    (:file "InvKinematics" :depends-on ("_package"))
    (:file "_package_InvKinematics" :depends-on ("_package"))
    (:file "GetRotMat" :depends-on ("_package"))
    (:file "_package_GetRotMat" :depends-on ("_package"))
    (:file "ResetMotion" :depends-on ("_package"))
    (:file "_package_ResetMotion" :depends-on ("_package"))
    (:file "GetCartesian" :depends-on ("_package"))
    (:file "_package_GetCartesian" :depends-on ("_package"))
    (:file "FwdKinematics" :depends-on ("_package"))
    (:file "_package_FwdKinematics" :depends-on ("_package"))
    (:file "GetJoints" :depends-on ("_package"))
    (:file "_package_GetJoints" :depends-on ("_package"))
    ))
