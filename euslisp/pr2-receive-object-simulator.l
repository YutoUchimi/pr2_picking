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


;; flush IRT-viewer based on real robot angle-vector
(defun viewer()
  (send *irtviewer* :draw-objects :flush t))


;; reset pose and stop grasp
(defun init()
  (send *pr2* :reset-pose)
  (send *irtviewer* :draw-objects :flush t)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 10000)
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

  (setq maxpoint-x (aref (send (send *box* :box) :maxpoint) 0))
  (setq maxpoint-y (aref (send (send *box* :box) :maxpoint) 1))
  (setq maxpoint-z (aref (send (send *box* :box) :maxpoint) 2))

  (setq minpoint-x (aref (send (send *box* :box) :minpoint) 0))
  (setq minpoint-y (aref (send (send *box* :box) :minpoint) 1))
  (setq minpoint-z (aref (send (send *box* :box) :minpoint) 2))

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
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000 :head-controller)
  (send *ri* :wait-interpolation :head-controller)
  (viewer))


;; search for a target object by swinging his head.
(defun search-for-object()
  (recog-target)
  (let ((flag t))
    (if flag
        (if t ;;(and center box)
            (progn (look-at) (setq flag nil) (format t "111~%"))
          (progn (send *pr2* :head :neck-y :joint-angle 30) ;; see left
                 (viewer)
                 (send *ri* :angle-vector (send *pr2* :angle-vector) 1000 :head-controller)
                 (recog-target))))

    (if flag
        (if t ;;(and center box)
            (progn (look-at) (setq flag nil) (format t "222~%"))
          (progn (send *pr2* :head :neck-y :joint-angle -30) ;; see right
                 (viewer)
                 (send *ri* :angle-vector (send *pr2* :angle-vector) 1000 :head-controller)
                 (recog-target))))

    (if flag
        (if t ;;(and center box)
            (progn (look-at) (setq flag nil) (format t "333~%"))
          (progn (send *pr2* :head :neck-p :joint-angle -30)
                 (send *pr2* :head :neck-y :joint-angle 0) ;; see up
                 (viewer)
                 (send *ri* :angle-vector (send *pr2* :angle-vector) 1000 :head-controller)
                 (recog-target))))

    (if flag
        (if t ;;(and center box)
            (progn (look-at) (setq flag nil) (format t "444~%"))
          (progn (send *pr2* :head :neck-p :joint-angle 30) ;; see down
                 (viewer)
                 (send *ri* :angle-vector (send *pr2* :angle-vector) 1000 :head-controller)
                 (recog-target))))

    (if flag
        (if t ;;(and center box)
            (progn (look-at) (setq flag nil))
          (progn (send *pr2* :head :neck-p :joint-angle 0) ;; see front
                 (viewer)
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
              (make-coords :pos (v+ (float-vector center-x
                                                  (/ (+ center-y maxpoint-y) 2)
                                                  minpoint-z)
                                    (float-vector 0 0 -100))
                           :rpy (float-vector 0 0 pi))
              :debug-view nil)
        (send *pr2* :rarm :inverse-kinematics
              (make-coords :pos (v+ (float-vector center-x
                                                  (/ (+ center-y minpoint-y) 2)
                                                  minpoint-z)
                                    (float-vector 0 0 -100))
                           :rpy (float-vector 0 0 pi))
              :debug-view nil)
        (viewer)
        (move 8000)

        ;; move both hands to the very place where the target object exists
        (send *pr2* :larm :inverse-kinematics
              (make-coords :pos (v+ (float-vector center-x
                                                  (/ (+ center-y maxpoint-y) 2)
                                                  minpoint-z)
                                    (float-vector 0 0 -50))
                           :rpy (float-vector 0 0 pi))
              :debug-view nil)
        (send *pr2* :rarm :inverse-kinematics
              (make-coords :pos (v+ (float-vector center-x
                                                  (/ (+ center-y minpoint-y) 2)
                                                  minpoint-z)
                                    (float-vector 0 0 -50))
                           :rpy (float-vector 0 0 pi))
              :debug-view nil)
        (viewer)
        (move 4000)

        ;; hold the target object in front of PR2
        (send *pr2* :init-pose)
        (send *pr2* :l_shoulder_lift_joint :joint-angle 50)
        (send *pr2* :l_elbow_flex_joint :joint-angle -75)
        (send *pr2* :l_wrist_roll_joint :joint-angle 180)
        (send *pr2* :r_shoulder_lift_joint :joint-angle 50)
        (send *pr2* :r_elbow_flex_joint :joint-angle -75)
        (send *pr2* :r_wrist_roll_joint :joint-angle 180)
        (viewer)
        (move 5000)

        (init))))
