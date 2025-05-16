(define (problem ServiceRobot-problem-b2r2i3)
(:domain ServiceRobot)
(:objects 
r1 r2 - room
b1 b2 - robot
i1 i2 i3 - item
h1-l h1-r h2-l h2-r - hand
)
(:init 
(at-robot b1 r1)
(has-hand b1 h1-l)
(has-hand b1 h1-r)
(free h1-l)
(free h1-r)
(at-robot b2 r1)
(has-hand b2 h2-l)
(has-hand b2 h2-r)
(free h2-l)
(free h2-r)
(at-item i1 r1)
(notbroken i1)
(at-item i2 r2)
(fragile i2)
(notbroken i2)
(at-item i3 r2)
(notbroken i3)
)
(:goal (and 
(at-item i1 r2)
(notbroken i1)
(at-item i2 r1)
(notbroken i2)
(at-item i3 r1)
(notbroken i3)
))
)
