(define (problem HomeRobot-problem1)
(:domain HomeRobot)
(:objects 
r1 r2 r3 r4 - room
h1 h2 - human
)
(:init
(unprepared r1)
(unprepared r2)
(unprepared r3)
(unprepared r4)
(at-robot r1)
(at-human h1 r2)
(at-human h2 r1)
(free-corridor)
)

(:goal (and (prepared r1)
	    (prepared r2)
            (prepared r3)
	    (prepared r4)
	    )
)
)
