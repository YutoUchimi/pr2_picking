#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_recognition_msgs")
(ros::load-ros-manifest "geometry_msgs")
(ros::load-ros-manifest "sensor_msgs")
(ros::load-ros-manifest "std_msgs")
(ros::load-ros-manifest "roseus")
(require "package://pr2eus_moveit/euslisp/collision-object-publisher.l")
(require "package://pr2eus/pr2-interface.l")
(ros::roseus "picking_manipulation_node")
(ros::advertise "stop_moveit" std_msgs::string 100)

(setq str_msg (instance std_msgs::string :init))
(setq *co* (instance collision-object-publisher :init))

(pr2-init t)


(defun init()
  (send *pr2* :reset-pose)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
  (send *ri* :stop-grasp)
  (send *ri* :wait-interpolation)
  (send str_msg :data "start"))


;; recognize position of the target object.
;; frame_id : /base_link
(defun recog-target()
  (setq boxes_msg
        (one-shot-subscribe
         "/cluster_indices_decomposer_target/boxes"
         jsk_recognition_msgs::BoundingBoxArray))

  (setq centroids
        (one-shot-subscribe
         "/cluster_indices_decomposer_target/centroid_pose_array"
         geometry_msgs::PoseArray))

  (setq box (car (send boxes_msg :boxes)))
  (setq center (car (send centroids :poses)))

  (if center
      (progn (setq center-x (* 1000 (send (send center :position) :x)))
             (setq center-y (* 1000 (send (send center :position) :y))))
    nil)
  (if box
      (setq center-z (* 1000 (send (send (send box :pose) :position):z)))
    ;;(format t "cannot find target!~%")
    ))


;; look at the target object if PR2 has already found it.
(defun look-at()
  (setq head-x (aref (send (send *pr2* :head-end-coords) :worldpos) 0))
  (setq head-y (aref (send (send *pr2* :head-end-coords) :worldpos) 1))
  (setq head-z (aref (send (send *pr2* :head-end-coords) :worldpos) 2))
  (send *pr2* :head :neck-y :joint-angle
        (rad2deg (atan (/ (- center-y head-y) center-x))))
  (send *pr2* :head :neck-p :joint-angle
        (rad2deg (atan (/ (- head-z center-z) center-x))))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000 :head-controller)
  (send *ri* :wait-interpolation :head-controller))


;; search for a target object by swinging his head.
(defun search-for-object()
  (recog-target)
  (let ((flag t))
    (if flag
        (if (and center box)
            (progn (look-at) (setq flag nil) (format t "111~%"))
          (progn (send *pr2* :head :neck-y :joint-angle 30) ;; see left
                 (send *ri* :angle-vector (send *pr2* :angle-vector) 1000 :head-controller)
                 (recog-target))))

    (if flag
        (if (and center box)
            (progn (look-at) (setq flag nil) (format t "222~%"))
          (progn (send *pr2* :head :neck-y :joint-angle -30) ;; see right
                 (send *ri* :angle-vector (send *pr2* :angle-vector) 1000 :head-controller)
                 (recog-target))))

    (if flag
        (if (and center box)
            (progn (look-at) (setq flag nil) (format t "333~%"))
          (progn (send *pr2* :head :neck-p :joint-angle -30)
                 (send *pr2* :head :neck-y :joint-angle 0) ;; see up
                 (send *ri* :angle-vector (send *pr2* :angle-vector) 1000 :head-controller)
                 (recog-target))))

    (if flag
        (if (and center box)
            (progn (look-at) (setq flag nil) (format t "444~%"))
          (progn (send *pr2* :head :neck-p :joint-angle 30) ;; see down
                 (send *ri* :angle-vector (send *pr2* :angle-vector) 1000 :head-controller)
                 (recog-target))))

    (if flag
        (if (and center box)
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


(defun pick()
  (init)
  (ros::publish "stop_moveit" str_msg)

  (if (search-for-object)
      (progn
        (send str_msg :data "stop")
        (ros::publish "stop_moveit" str_msg)

        (send *pr2* :larm :inverse-kinematics
              (make-coords :pos (v+ (float-vector center-x center-y center-z)
                                    #f(0 0 -200)))
                    :debug-view nil)
        (move 10000)

        (send *pr2* :larm :inverse-kinematics
              (make-coords :pos (float-vector center-x center-y center-z))
                    :debug-view nil)
        (move 5000)

        (send *ri* :start-grasp :larm)
        (send *ri* :wait-interpolation)

        (send *pr2* :larm :inverse-kinematics
              (make-coords :pos (v+ (float-vector center-x center-y center-z)
                                    #f(0 0 -50)))
                    :debug-view nil)
        (move 3000)

        (send *pr2* :larm :inverse-kinematics
              (make-coords :pos (float-vector 350 600 (+ center-z 100)))
                    :debug-view nil)
        (move 5000)

        (send *pr2* :larm :inverse-kinematics
              (make-coords :pos (float-vector 350 600 100))
                    :debug-view nil
                    :rotation-axis nil)
        (move 5000)

        (send *ri* :stop-grasp :larm)

        (init))))
