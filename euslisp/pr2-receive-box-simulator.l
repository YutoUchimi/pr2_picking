#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_recognition_msgs")
(ros::load-ros-manifest "geometry_msgs")
(ros::load-ros-manifest "sensor_msgs")
(ros::load-ros-manifest "std_msgs")
(ros::load-ros-manifest "roseus")
(require "package://pr2eus_moveit/euslisp/collision-object-publisher.l")
(require "package://pr2eus/pr2-interface.l")

(ros::roseus "receive_object")
(ros::advertise "stop_moveit" std_msgs::string 100)

(setq str_msg (instance std_msgs::string :init))
(setq *co* (instance collision-object-publisher :init))

(pr2-init t) ;; reset pose

(setq *box* (make-cube 300 500 200))
(send *box* :locate #f(600 200 800))
(send *box* :set-color #f(1.0 0.6 0.0))
(objects (list *pr2* *box*))


;; reset pose and stop grasp
(defun init()
  (send *pr2* :reset-pose)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
  (send *ri* :stop-grasp)
  (send *ri* :wait-interpolation)
  (send str_msg :data "start"))


;; recognize position of the target object.
;; frame_id : /base_link
(defun recog-target()
  ;; (setq boxes_msg
  ;;       (one-shot-subscribe
  ;;        "/cluster_indices_decomposer_target/boxes"
  ;;        jsk_recognition_msgs::BoundingBoxArray))

  ;; (setq centroids
  ;;       (one-shot-subscribe
  ;;        "/cluster_indices_decomposer_target/centroid_pose_array"
  ;;        geometry_msgs::PoseArray))

  ;; (setq box (car (send boxes_msg :boxes)))
  ;; (setq center (car (send centroids :poses)))

  (setq center-x (aref (send *box* :centroid) 0))
  (setq center-y (aref (send *box* :centroid) 1))
  (setq center-z (aref (send *box* :centroid) 2))

  ;; (if center
  ;;     (progn (setq center-x (* 1000 (send (send center :position) :x)))
  ;;            (setq center-y (* 1000 (send (send center :position) :y))))
  ;;   nil)
  ;; (if box
  ;;     (setq center-z (* 1000 (send (send (send box :pose) :position):z)))
  ;;   ;;(format t "cannot find target!~%")
  ;;   )
  )


;; look at the target object if PR2 has already found it. (require recog-target)
(defun look-at()
  (setq head-x (aref (send (send *pr2* :head-end-coords) :worldpos) 0))
  (setq head-y (aref (send (send *pr2* :head-end-coords) :worldpos) 1))
  (setq head-z (aref (send (send *pr2* :head-end-coords) :worldpos) 2))
  (send *pr2* :head :neck-y :joint-angle
        (rad2deg (atan (/ (- center-y head-y) center-x))))
  (send *pr2* :head :neck-p :joint-angle
        (rad2deg (atan (/ (- head-z center-z) center-x))))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000 :head-controller)
  (send *ri* :wait-interpolation :head-controller)
  (objects (list *pr2* *box*))


;; search for a target object by swinging his head.
(defun search-for-object()
  (recog-target)
  (let ((flag t))
    (if flag
        (if t ;;(and center box)
            (progn (look-at) (setq flag nil) (format t "111~%"))
          (progn (send *pr2* :head :neck-y :joint-angle 30) ;; see left
                 (send *ri* :angle-vector (send *pr2* :angle-vector) 1000 :head-controller)
                 (recog-target))))

    (if flag
        (if t ;;(and center box)
            (progn (look-at) (setq flag nil) (format t "222~%"))
          (progn (send *pr2* :head :neck-y :joint-angle -30) ;; see right
                 (send *ri* :angle-vector (send *pr2* :angle-vector) 1000 :head-controller)
                 (recog-target))))

    (if flag
        (if t ;;(and center box)
            (progn (look-at) (setq flag nil) (format t "333~%"))
          (progn (send *pr2* :head :neck-p :joint-angle -30)
                 (send *pr2* :head :neck-y :joint-angle 0) ;; see up
                 (send *ri* :angle-vector (send *pr2* :angle-vector) 1000 :head-controller)
                 (recog-target))))

    (if flag
        (if t ;;(and center box)
            (progn (look-at) (setq flag nil) (format t "444~%"))
          (progn (send *pr2* :head :neck-p :joint-angle 30) ;; see down
                 (send *ri* :angle-vector (send *pr2* :angle-vector) 1000 :head-controller)
                 (recog-target))))

    (if flag
        (if t ;;(and center box)
            (progn (look-at) (setq flag nil))
          (progn (send *pr2* :head :neck-p :joint-angle 0) ;; see front
                 (send *ri* :angle-vector (send *pr2* :angle-vector) 1000 :head-controller)
                 (recog-target))))

    (if (null flag)
        (progn (format t "find object!~%") t)
      (progn (format t "cannot find object!~%") nil))))


;; order the real PR2 robot to follow the joint angle of the model robot
(defun move (time &optional (trial-number 100))
  (let ((test nil) (i 0))
    (while (and (null test) (< i trial-number))
      (setq i (+ i 1))
      (setq test (send *ri* :angle-vector (send *pr2* :angle-vector) time)))
    (send *ri* :wait-interpolation)))


(defun receive()
  (init)
  (ros::publish "stop_moveit" str_msg)

  (if (search-for-object)
      (progn
        (send str_msg :data "stop")
        (ros::publish "stop_moveit" str_msg)

        ;; move both hands to under the detected target object
        (send *pr2* :larm :inverse-kinematics
              (make-coords :pos (v+ (float-vector center-x center-y center-z)
                                    #f(100 100 -200))
                           :rpy (float-vector 0 (deg2rad -45) 0))
              :debug-view nil)
        (send *pr2* :rarm :inverse-kinematics
              (make-coords :pos (v+ (float-vector center-x center-y center-z)
                                    #f(100 -100 -200))
                           :rpy (float-vector 0 (deg2rad -45) 0))
              :debug-view nil)
        (move 10000)

        ;; move both hands to the very place where the target object exists
        (send *pr2* :larm :inverse-kinematics
              (make-coords :pos (v+ (float-vector center-x center-y center-z)
                                    #f(100 100 0))
                           :rpy (float-vector 0 (deg2rad -45) 0))
              :debug-view nil)
        (send *pr2* :rarm :inverse-kinematics
              (make-coords :pos (v+ (float-vector center-x center-y center-z)
                                    #f(100 -100 0))
                           :rpy (float-vector 0 (deg2rad -45) 0))
              :debug-view nil)
        (move 3000)

        ;; hold the target object in front of PR2
        (send *pr2* :init-pose)
        (send *pr2* :l_shoulder_lift_joint :joint-angle 50)
        (send *pr2* :l_elbow_flex_joint :joint-angle -75)
        (send *pr2* :r_shoulder_lift_joint :joint-angle 50)
        (send *pr2* :r_elbow_flex_joint :joint-angle -75)
        (move 5000)

        (init))))
