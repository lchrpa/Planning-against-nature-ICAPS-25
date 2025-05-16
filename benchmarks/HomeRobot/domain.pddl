(define (domain HomeRobot)
(:requirements :typing :equality)
(:types room human - object
)

(:predicates (at-robot ?r - room)
             (at-human ?h - human ?r - room)
             (at-robot-corridor)
             (at-human-corridor ?h - human)
             (free-corridor)
             (activity-done ?r - room)
             (unprepared ?r - room)
             (prepared ?r - room)
             
)

(:action move-robot-room
:parameters (?r - room)
:precondition (and (at-robot-corridor)
               )
                
:effect (and (not (at-robot-corridor))
             (free-corridor)
             (at-robot ?r)
        )
)


(:action move-robot-corridor
:parameters (?r - room)
:precondition (and (free-corridor)
                   (at-robot ?r)
               )
:effect (and (at-robot-corridor)
             (not (at-robot ?r))
             (not (free-corridor))
        )
)

(:action prepare-room
:parameters (?r - room)
:precondition (and (unprepared ?r)
                   (at-robot ?r)
              )
:effect (and (not (unprepared ?r))
             (prepared ?r)
        )
)

(:event move-human-corridor
:parameters (?h - human ?r - room)
:precondition (and (free-corridor)
                   (at-human ?h ?r)
               )
:effect (and (at-human-corridor ?h)
             (not (at-human ?h ?r))
             (not (free-corridor))
        )
)


(:event move-human-room
:parameters (?h - human ?r - room)
:precondition (and (at-human-corridor ?h)
              )
:effect (and (not (at-human-corridor ?h))
             (free-corridor)
             (at-human ?h ?r)
        )
)

;(:event perform-activity
;:parameters (?r - room)
;:precondition (and (prepared ?r)
;                   (at-human ?r)
;              )
;:effect (and (not (prepared ?r))
;             (activity-done ?r)           
;        )
;)
)
