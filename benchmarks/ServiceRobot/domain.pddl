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

(:event crack
:parameters (?h1 - hand ?h2 - hand ?b - robot ?i1 - item ?i2 - item)
:precondition (and (has-hand ?b ?h1)
                   (has-hand ?b ?h2)
                   (carries ?i1 ?h1)
                   (carries ?i2 ?h2)
                   (not (= ?h1 ?h2))
                   (not (= ?i1 ?i2))
                   (fragile ?i1)
               )
:effect (and (not (notbroken ?i1))
        )
)


(:event interfere
:parameters (?h - hand ?b1 - robot ?b2 - robot ?i - item)
:precondition (and (has-hand ?b1 ?h)
                   (carries ?i ?h)
                   (at-robot-corridor ?b1)
                   (at-robot-corridor ?b2)
                   (not (= ?b1 ?b2))
                   (fragile ?i)
               )
:effect (and (not (notbroken ?i))
        )
)

)
