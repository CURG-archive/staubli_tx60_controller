; Auto-generated. Do not edit!


(in-package staubliTX60-srv)


;//! \htmlinclude GetRotMat-request.msg.html

(defclass <GetRotMat-request> (ros-message)
  ()
)
(defmethod serialize ((msg <GetRotMat-request>) ostream)
  "Serializes a message object of type '<GetRotMat-request>"
)
(defmethod deserialize ((msg <GetRotMat-request>) istream)
  "Deserializes a message object of type '<GetRotMat-request>"
  msg
)
(defmethod ros-datatype ((msg (eql '<GetRotMat-request>)))
  "Returns string type for a service object of type '<GetRotMat-request>"
  "staubliTX60/GetRotMatRequest")
(defmethod md5sum ((type (eql '<GetRotMat-request>)))
  "Returns md5sum for a message object of type '<GetRotMat-request>"
  "1f7818e7ce16454badf1bee936b0ff16")
(defmethod message-definition ((type (eql '<GetRotMat-request>)))
  "Returns full string definition for message of type '<GetRotMat-request>"
  (format nil "~%"))
(defmethod serialization-length ((msg <GetRotMat-request>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <GetRotMat-request>))
  "Converts a ROS message object to a list"
  (list '<GetRotMat-request>
))
;//! \htmlinclude GetRotMat-response.msg.html

(defclass <GetRotMat-response> (ros-message)
  ((m
    :reader m-val
    :initarg :m
    :type (vector float)
   :initform (make-array 0 :element-type 'float :initial-element 0.0)))
)
(defmethod serialize ((msg <GetRotMat-response>) ostream)
  "Serializes a message object of type '<GetRotMat-response>"
  (let ((__ros_arr_len (length (slot-value msg 'm))))
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
    (slot-value msg 'm))
)
(defmethod deserialize ((msg <GetRotMat-response>) istream)
  "Deserializes a message object of type '<GetRotMat-response>"
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'm) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'm)))
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
(defmethod ros-datatype ((msg (eql '<GetRotMat-response>)))
  "Returns string type for a service object of type '<GetRotMat-response>"
  "staubliTX60/GetRotMatResponse")
(defmethod md5sum ((type (eql '<GetRotMat-response>)))
  "Returns md5sum for a message object of type '<GetRotMat-response>"
  "1f7818e7ce16454badf1bee936b0ff16")
(defmethod message-definition ((type (eql '<GetRotMat-response>)))
  "Returns full string definition for message of type '<GetRotMat-response>"
  (format nil "float64[] m ~%~%~%"))
(defmethod serialization-length ((msg <GetRotMat-response>))
  (+ 0
     4 (reduce #'+ (slot-value msg 'm) :key #'(lambda (ele) (declare (ignorable ele)) (+ 8)))
))
(defmethod ros-message-to-list ((msg <GetRotMat-response>))
  "Converts a ROS message object to a list"
  (list '<GetRotMat-response>
    (cons ':m (m-val msg))
))
(defmethod service-request-type ((msg (eql 'GetRotMat)))
  '<GetRotMat-request>)
(defmethod service-response-type ((msg (eql 'GetRotMat)))
  '<GetRotMat-response>)
(defmethod ros-datatype ((msg (eql 'GetRotMat)))
  "Returns string type for a service object of type '<GetRotMat>"
  "staubliTX60/GetRotMat")
