; Auto-generated. Do not edit!


(in-package staubliTX60-srv)


;//! \htmlinclude InvKinematics-request.msg.html

(defclass <InvKinematics-request> (ros-message)
  ((x
    :reader x-val
    :initarg :x
    :type float
    :initform 0.0)
   (y
    :reader y-val
    :initarg :y
    :type float
    :initform 0.0)
   (z
    :reader z-val
    :initarg :z
    :type float
    :initform 0.0)
   (rx
    :reader rx-val
    :initarg :rx
    :type float
    :initform 0.0)
   (ry
    :reader ry-val
    :initarg :ry
    :type float
    :initform 0.0)
   (rz
    :reader rz-val
    :initarg :rz
    :type float
    :initform 0.0))
)
(defmethod serialize ((msg <InvKinematics-request>) ostream)
  "Serializes a message object of type '<InvKinematics-request>"
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'x))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'y))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'z))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'rx))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'ry))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'rz))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
)
(defmethod deserialize ((msg <InvKinematics-request>) istream)
  "Deserializes a message object of type '<InvKinematics-request>"
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'x) (roslisp-utils:decode-double-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'y) (roslisp-utils:decode-double-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'z) (roslisp-utils:decode-double-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'rx) (roslisp-utils:decode-double-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'ry) (roslisp-utils:decode-double-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'rz) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<InvKinematics-request>)))
  "Returns string type for a service object of type '<InvKinematics-request>"
  "staubliTX60/InvKinematicsRequest")
(defmethod md5sum ((type (eql '<InvKinematics-request>)))
  "Returns md5sum for a message object of type '<InvKinematics-request>"
  "67595675823d506b00f163740b4f2c36")
(defmethod message-definition ((type (eql '<InvKinematics-request>)))
  "Returns full string definition for message of type '<InvKinematics-request>"
  (format nil "float64 x~%float64 y~%float64 z~%float64 rx~%float64 ry~%float64 rz~%~%"))
(defmethod serialization-length ((msg <InvKinematics-request>))
  (+ 0
     8
     8
     8
     8
     8
     8
))
(defmethod ros-message-to-list ((msg <InvKinematics-request>))
  "Converts a ROS message object to a list"
  (list '<InvKinematics-request>
    (cons ':x (x-val msg))
    (cons ':y (y-val msg))
    (cons ':z (z-val msg))
    (cons ':rx (rx-val msg))
    (cons ':ry (ry-val msg))
    (cons ':rz (rz-val msg))
))
;//! \htmlinclude InvKinematics-response.msg.html

(defclass <InvKinematics-response> (ros-message)
  ((j
    :reader j-val
    :initarg :j
    :type (vector float)
   :initform (make-array 0 :element-type 'float :initial-element 0.0)))
)
(defmethod serialize ((msg <InvKinematics-response>) ostream)
  "Serializes a message object of type '<InvKinematics-response>"
  (let ((__ros_arr_len (length (slot-value msg 'j))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele) (let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream)))
    (slot-value msg 'j))
)
(defmethod deserialize ((msg <InvKinematics-response>) istream)
  "Deserializes a message object of type '<InvKinematics-response>"
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'j) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'j)))
      (dotimes (i __ros_arr_len)
(let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(defmethod ros-datatype ((msg (eql '<InvKinematics-response>)))
  "Returns string type for a service object of type '<InvKinematics-response>"
  "staubliTX60/InvKinematicsResponse")
(defmethod md5sum ((type (eql '<InvKinematics-response>)))
  "Returns md5sum for a message object of type '<InvKinematics-response>"
  "67595675823d506b00f163740b4f2c36")
(defmethod message-definition ((type (eql '<InvKinematics-response>)))
  "Returns full string definition for message of type '<InvKinematics-response>"
  (format nil "float64[] j~%~%~%"))
(defmethod serialization-length ((msg <InvKinematics-response>))
  (+ 0
     4 (reduce #'+ (slot-value msg 'j) :key #'(lambda (ele) (declare (ignorable ele)) (+ 8)))
))
(defmethod ros-message-to-list ((msg <InvKinematics-response>))
  "Converts a ROS message object to a list"
  (list '<InvKinematics-response>
    (cons ':j (j-val msg))
))
(defmethod service-request-type ((msg (eql 'InvKinematics)))
  '<InvKinematics-request>)
(defmethod service-response-type ((msg (eql 'InvKinematics)))
  '<InvKinematics-response>)
(defmethod ros-datatype ((msg (eql 'InvKinematics)))
  "Returns string type for a service object of type '<InvKinematics>"
  "staubliTX60/InvKinematics")
