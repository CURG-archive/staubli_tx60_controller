; Auto-generated. Do not edit!


(in-package staubli_tx60-srv)


;//! \htmlinclude GetJoints-request.msg.html

(defclass <GetJoints-request> (ros-message)
  ()
)
(defmethod serialize ((msg <GetJoints-request>) ostream)
  "Serializes a message object of type '<GetJoints-request>"
)
(defmethod deserialize ((msg <GetJoints-request>) istream)
  "Deserializes a message object of type '<GetJoints-request>"
  msg
)
(defmethod ros-datatype ((msg (eql '<GetJoints-request>)))
  "Returns string type for a service object of type '<GetJoints-request>"
  "staubli_tx60/GetJointsRequest")
(defmethod md5sum ((type (eql '<GetJoints-request>)))
  "Returns md5sum for a message object of type '<GetJoints-request>"
  "59542e81b1fd2eaee58892b9055022e8")
(defmethod message-definition ((type (eql '<GetJoints-request>)))
  "Returns full string definition for message of type '<GetJoints-request>"
  (format nil "~%"))
(defmethod serialization-length ((msg <GetJoints-request>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <GetJoints-request>))
  "Converts a ROS message object to a list"
  (list '<GetJoints-request>
))
;//! \htmlinclude GetJoints-response.msg.html

(defclass <GetJoints-response> (ros-message)
  ((j
    :reader j-val
    :initarg :j
    :type (vector float)
   :initform (make-array 0 :element-type 'float :initial-element 0.0)))
)
(defmethod serialize ((msg <GetJoints-response>) ostream)
  "Serializes a message object of type '<GetJoints-response>"
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
(defmethod deserialize ((msg <GetJoints-response>) istream)
  "Deserializes a message object of type '<GetJoints-response>"
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
(defmethod ros-datatype ((msg (eql '<GetJoints-response>)))
  "Returns string type for a service object of type '<GetJoints-response>"
  "staubli_tx60/GetJointsResponse")
(defmethod md5sum ((type (eql '<GetJoints-response>)))
  "Returns md5sum for a message object of type '<GetJoints-response>"
  "59542e81b1fd2eaee58892b9055022e8")
(defmethod message-definition ((type (eql '<GetJoints-response>)))
  "Returns full string definition for message of type '<GetJoints-response>"
  (format nil "float64[] j~%~%~%"))
(defmethod serialization-length ((msg <GetJoints-response>))
  (+ 0
     4 (reduce #'+ (slot-value msg 'j) :key #'(lambda (ele) (declare (ignorable ele)) (+ 8)))
))
(defmethod ros-message-to-list ((msg <GetJoints-response>))
  "Converts a ROS message object to a list"
  (list '<GetJoints-response>
    (cons ':j (j-val msg))
))
(defmethod service-request-type ((msg (eql 'GetJoints)))
  '<GetJoints-request>)
(defmethod service-response-type ((msg (eql 'GetJoints)))
  '<GetJoints-response>)
(defmethod ros-datatype ((msg (eql 'GetJoints)))
  "Returns string type for a service object of type '<GetJoints>"
  "staubli_tx60/GetJoints")
