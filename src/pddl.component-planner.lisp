#|
  This file is a part of pddl.component-planner project.
  Copyright (c) 2013 guicho ()
|#

(in-package :pddl.component-planner)

(cl-syntax:use-syntax :annot)
;; blah blah blah.

@export
(defun build-component-problem (abstract-task)
  "build a small problem which contains only the relevant objects in a
abstract task."
  (ematch abstract-task
    ((abstract-component-task :problem *problem*)
     (ematch *problem*
       ((pddl-problem :name total-name
                      :domain *domain*
                      :objects objs
                      :init init
                      :metric metric)
        (match abstract-task
          ((abstract-component-task- (ac (abstract-component components))
                                     (goal task-goal))
           (let* ((removed nil)
                  (environment-objects
                   (remove-if
                     (lambda (o)
                       (when (some (lambda (comp)
                                     (pddl-supertype-p (type o) (type comp)))
                                   components)
                         (push o removed)
                         t))
                     (set-difference objs components))))
             (format t "~&Component: ~{~a~^, ~_~}" (mapcar #'name components))
             (format t "~&Removed  : ~{~a~^, ~_~}" (mapcar #'name removed))
             (pddl-problem
              :domain *domain*
              :name (apply #'concatenate-symbols
                           total-name 'component (mapcar #'name components))
              :objects (append components environment-objects)
              :init (remove-if
                     (lambda (f)
                       (some (lambda (p)
                               (find p removed))
                             (parameters f)))
                     init)
              :goal (list* 'and task-goal)
              :metric metric)))))))))

@export
(defcached plan-task (task)
  "Calls build-component-problem, make a plan with FD, then parse the results.
returns a PDDL-PLAN."
  (let* ((*problem* (build-component-problem task))
         (*domain* (domain *problem*)))
    (mapcar (let ((i 0))
              (curry #'pddl-plan
                     :name (concatenate-symbols (name *problem*) 'plan (incf i))
                     :path))
            (test-problem
             (write-problem *problem*)
             (path *domain*)
             :time-limit 3
             :hard-time-limit 1800
             :memory 500000 ;; 500MB
             ;; :options "--search astar(lmcut())"
             ))))

@export
(defun clear-plan-task-cache ()
  (clear-cache *PLAN-TASK-CACHE*))

@export
(defun task-plan-equal (t1 t2)
  ;; (assert (abstract-component-task-strict= t1 t2))
  (let* ((problem
          ;; @break+
          (build-component-problem t2))
         (problem-path
          ;; @break+
           (write-problem problem)))
    (some 
     (lambda (plan)
       (validate-plan
        (path (domain problem))
        problem-path
        (write-plan
         (apply-mapping plan (mapping-between-tasks t1 t2) problem))
        ;; :verbose t
        ))
     (plan-task t1))))

@export
(defun fluently-connected-objects (components attributes f)
  (let ((ps (parameters f)))
    (setf ps (set-difference ps components))
    (setf ps (set-difference ps attributes))
    ps))

@export
(defun goal-object (task)
  (ematch task
    ((abstract-component-task
      goal
      (ac (abstract-component
           components
           attributes)))
     (mappend 
      (curry #'fluently-connected-objects components attributes)
      goal))))

@export
(defun init-object (task)
  (ematch task
    ((abstract-component-task
      init
      (ac (abstract-component
           components
           attributes)))
     (mappend 
      (curry #'fluently-connected-objects components attributes)
      init))))

@export
(defun mapping-between-tasks (t1 t2)
  (list (two-list-mapping (abstract-component-components
                           (abstract-component-task-ac t1))
                          (abstract-component-components
                           (abstract-component-task-ac t2)))
        (two-list-mapping (init-object t1) (init-object t2))
        (two-list-mapping (goal-object t1) (goal-object t2))))

@export
(defun two-list-mapping (os1 os2)
  (mapcar (lambda (o1 o2)
            (cons o1 o2))
          os1 os2))


@export
(defun apply-mapping (plan mapping mapped-problem)
  (ematch plan
    ((pddl-plan actions problem)
     (let ((%mapping 
            ;; @break+
            (%remove-nochange mapping)))
       (shallow-copy
        plan
        :problem mapped-problem
        :name (concatenate-symbols
               'plan-mapped-from
               (name plan)
               'to
               (name mapped-problem))
        :actions
        ;;@break+
        (map 'vector
             (lambda (action)
               (ematch action
                 ((or (pddl-initial-action)
                      (pddl-goal-action))
                  action)
                 ((pddl-intermediate-action parameters)
                  (shallow-copy
                   action
                   :parameters (%apply parameters %mapping)))))
             actions))))))

(defun %apply (parameters mappings)
  (reduce (lambda (parameters mapping)
            (ematch mapping
              ((cons from to)
               (substitute to from parameters))))
          mappings :initial-value parameters))

(defun %remove-nochange (mapping)
  (ematch mapping
    ((list component-mapping init-mapping goal-mapping)
     (flet ((fn (mapping)
              (ematch mapping
                ((cons from to)
                 (eq from to)))))
       (append component-mapping
               (remove-if #'fn init-mapping)
               (remove-if #'fn goal-mapping))))))

