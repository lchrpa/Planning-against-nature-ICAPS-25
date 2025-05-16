(define (domain ServiceRobot)
(:requirements :typing :equality)
(:types robot hand room item - object
)

(:predicates (at-robot ?b - robot ?r - room)
             (at-item ?i - item ?r - room)
             (at-robot-corridor ?b - robot)
             (has-hand ?b - robot ?h - hand)
             (free ?h - hand)
             (carries ?i - item ?h - hand)
             (fragile ?i - item)
             (notbroken ?i - item)
             
)

(:action move-robot-room
:parameters (?b - robot ?r - room)
:precondition (and (at-robot-corridor ?b)
               )
                
:effect (and (not (at-robot-corridor ?b))
             (at-robot ?b ?r)
        )
)


(:action move-robot-corridor
:parameters (?b - robot ?r - room)
:precondition (and 
                   (at-robot ?b ?r)
               )
:effect (and (at-robot-corridor ?b)
             (not (at-robot ?b ?r))
        )
)

(:action pickup
:parameters (?b - robot ?r - room ?h - hand ?i - item)
:precondition (and (has-hand ?b ?h)
                   (at-robot ?b ?r)
                   (free ?h)
                   (at-item ?i ?r)
              )
:effect (and (not (free ?h))
             (not (at-item ?i ?r))
             (carries ?i ?h)
        )
)

(:action putdown
:parameters (?b - robot ?r - room ?h - hand ?i - item)
:precondition (and (has-hand ?b ?h)
                   (at-robot ?b ?r)
                   (carries ?i ?h)
                 
              )
:effect (and (free ?h)
             (at-item ?i ?r)
             (not (carries ?i ?h))
        )
)

)
