
(in-package :pddl.component-planner)

#|

given a set of grounded macros -> 

find the shortest path to the entrance of next macro

then add it as another macro

|#

(defun reverse-problem (gmacro task)
  (match task
    ((abstract-component-task problem init goal)
     (pddl-problem :name (symbolicate 'rev- (name gmacro))
                   :domain (domain problem)
                   :objects (objects/const problem)
                   :metric (metric problem)
                   :init (apply-ground-action gmacro (init problem))
                   :goal `(and ,@goal ;; maintain the previously achieved goal is true
                               ,@(set-difference
                                  (remove-if
                                   (rcurry #'typep 'pddl-function-state)
                                   (init problem))
                                  init ;; remove the fluents regarding this task
                                  :test #'eqstate))))))

(defun %solve-rev (problem)
  (let* ((dir (mktemp "reverse-problem" t))
         (domain (domain problem)))
    (when (within-time-limit)
      (multiple-value-match
          (funcall #'test-problem-common
                   (write-pddl (if *remove-component-problem-cost*
                                   (remove-costs problem)
                                   problem)
                               "problem.pddl" dir)
                   (write-pddl (if *remove-component-problem-cost*
                                   (remove-costs domain)
                                   domain)
                               "domain.pddl" dir)
                   :name *preprocessor*
                   :options *preprocessor-options*
                   :time-limit 1
                   :hard-time-limit *component-plan-time-limit*
                   :memory *memory-limit*
                   :verbose *debug-preprocessing*)
        ((plans time memory _)
         (signal 'evaluation-signal :usage (list time memory))
         (when plans
           (pddl-plan
            :domain domain
            :problem problem
            :name (concatenate-symbols
                   (name problem) 'plan)
            :path (first plans))))))))

#+nil
(defun reverse-macro (bmvector)
  "deprecated. the precondition does not contain restriction --> bloat"
  (multiple-value-bind (gms tasks) (get-actions-grounded bmvector)
    (when-let ((plan (%solve-rev
                      (reverse-problem (first gms) (first tasks)))))
      ;; reuse this function
      (ematch (bmvector (vector tasks plan)) ;; -> (vector tasks macro)
        ((and it (vector _ (macro-action (name (place name)))))
         (setf name (symbolicate 'rev- name))
         it)))))

(defun cyclic-macro (bmvector)
  "grounded by default."
  (handler-bind ((warning #'muffle-warning))
    (multiple-value-bind (gms tasks) (get-actions-grounded bmvector)
      (restart-case
          (if-let ((plan (%solve-rev
                          (reverse-problem (first gms) (first tasks)))))
            (handler-case
                (let ((reverse-gms
                       (get-actions-grounded
                        (bmvector (vector tasks plan)))))
                  (values
                   (mapcar
                    (lambda (gm rgm)
                      (change-class
                       (merge-ground-actions gm rgm)
                       'ground-macro-action
                       :parameters nil
                       :actions (concatenate 'vector (actions gm) (actions rgm))
                       :name
                       (let ((str (symbol-name
                                   (symbolicate
                                    'cycle- (symbol-name (name gm))))))
                         (if (< 30 (length str))
                             (gensym (subseq str 0 29))
                             (gensym str)))))
                    gms reverse-gms)
                   tasks))
              (zero-length-plan () (invoke-restart 'fail)))
            (invoke-restart 'fail))
        (fail ()
          (format t "~& No reverse macro found in this task.")
          (values gms tasks))))))




