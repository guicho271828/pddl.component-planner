
(handler-bind ((SB-INT:EXTENSION-FAILURE
                (lambda (c)
                  (invoke-restart (find-restart 'asdf/action:ACCEPT c)))))
  (asdf:load-system :pddl.component-planner-test))

(in-package :pddl.component-planner-test)
(defparameter *delayed-problems*
              (append (load-and-collect-problems
                       '(:pddl.instances.barman-x1.3)
                       '(:cocktail)
                       ".*BARMAN-.*")
                      (load-and-collect-problems
                       '(:pddl.instances.cell-assembly-eachparts)
                       '(:base)
                       "CELL-ASSEMBLY-(2A2B-MIXED-EACH-.*|MODEL2A-EACH-[12][0-9])")
                      (load-and-collect-problems
                       '(:pddl.instances.elevators)
                       '(:passenger)
                       ".*ELEVATORS.*")
                      (load-and-collect-problems
                       '(:pddl.instances.openstacks)
                       '(:order :product)
                       ".*OPENSTACKS.*")
                      (load-and-collect-problems
                       '(:pddl.instances.rover)
                       '(:objective)
                       "ROVERPROB[0-9]*")
                      (load-and-collect-problems
                       '(:pddl.instances.woodworking-large
                         :pddl.instances.woodworking-xlarge)
                       '(:part)
                       "WOOD-PROB-SAT-[0-9]*")
                      (load-and-collect-problems
                       '(:pddl.instances.satellite-typed)
                       '(:direction)
                       "SATELLITE-TYPED-.*")
                      (load-and-collect-problems
                       '(:pddl.instances.barman-x1.3)
                       '(:shot)
                       ".*BARMAN-.*")))

(defvar *pnum* (length *delayed-problems*))

(print "This is script.lisp")

(defun run-parent (parallel i)
  (iter (for j from i below *pnum* by parallel)
        (with-output-to-file
            (s (merge-pathnames
                (format nil "log-~a-~a" i j)
                *log-dir*)
               :if-does-not-exist :create
               :if-exists :supersede)
          (for process = 
               (sb-ext:run-program
                (merge-pathnames "lispimage" *default-pathname-defaults*)
                (list "--dynamic-space-size" "15000"
                      (princ-to-string j))
                :search t
                :output s))
          (when (not (= 0 (sb-ext:process-exit-code process)))
            (format s "The child sbcl process has stopped with some error!")))))


(defun run-parent-dry (parallel i)
  (iter (for j from i below *pnum* by parallel)
        (format t "~&Running a new process with parallel = ~a, i = ~a, j = ~a~&"
                parallel i j)
        (for process = 
             (sb-ext:run-program
              (merge-pathnames "lispimage" *default-pathname-defaults*)
              (list "--dynamic-space-size" "15500"
                    "-d"
                    (princ-to-string j))
              :output t
              :search t))))

(defun run-children-dry (j)
  (print (sb-ext:dynamic-space-size))
  (format t "~&In child process ~a~&" j))

(defun make-image (&optional repl)
  (print (bt:all-threads))
  (lparallel:end-kernel :wait t)
  (sb-ext:gc :full t)
  (if repl
      (sb-ext:save-lisp-and-die "repl-image"
                                :executable t
                                :purify t)
      (sb-ext:save-lisp-and-die "lispimage"
                                :toplevel #'main
                                :executable t
                                :purify t)))

(defun main ()
  (print sb-ext:*posix-argv*)
  (match sb-ext:*posix-argv*
    ((list _ "image")
     (make-image))
    ((list _ "repl")
     (make-image t))
    ((list _ "-d" parallel i)
     (run-parent-dry (parse-integer parallel)
                     (parse-integer i)))
    ((list _ "-d" j)
     (run-children-dry (parse-integer j)))
    ((list _ parallel i)
     (run-parent (parse-integer parallel)
                 (parse-integer i)))
    ((list _ j)
     (sb-ext:disable-debugger)
     (benchmark (parse-integer j)))))

(defun reload ()
  (load "script.lisp"))

(defun reload-save-repl ()
  (reload)
  (make-image t))

(defun reload-save-image ()
  (reload)
  (make-image))

(main)
