(define (problem ServiceRobot-problem-b1r2i2)
(:domain ServiceRobot)
(:objects 
r1 r2 - room
b1 - robot
i1 i2 - item
h1-l h1-r - hand
)
(:init 
(at-robot b1 r2)
(has-hand b1 h1-l)
(has-hand b1 h1-r)
(free h1-l)
(free h1-r)
(at-item i1 r1)
(notbroken i1)
(at-item i2 r2)
(notbroken i2)
)
(:goal (and 
(at-item i1 r2)
(notbroken i1)
(at-item i2 r1)
(notbroken i2)
))
)
