

(ql:quickload :pddl.component-planner.experiment)
(sb-profile:profile "PDDL" "PDDL.COMPONENT-PLANNER" "PDDL.COMPONENT-ABSTRACTION" "GUICHO-UTILITIES" "EAZYLAZY")
(in-package "PDDL.COMPONENT-PLANNER.EXPERIMENT")
(time (pddl.component-planner.experiment::test1-1))
(time (pddl.component-planner.experiment::test1-2))
(time (pddl.component-planner.experiment::test1-3))
;;(time (pddl.component-planner.experiment::test1-4))
(time (pddl.component-planner.experiment::test2-1))
(time (pddl.component-planner.experiment::test2-2))
(time (pddl.component-planner.experiment::test2-3))
;;(time (pddl.component-planner.experiment::test2-4))
(sb-profile:report :print-no-call-list nil)
