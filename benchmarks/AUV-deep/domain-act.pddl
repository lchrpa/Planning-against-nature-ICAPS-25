(define (domain AUV)
(:requirements :typing :equality)
(:types vehicle location resource - object
        auv ship - vehicle
)

(:predicates (at ?v - vehicle ?l - location)
             (connected ?l1 ?l2 - location)
             (at-res ?r - resource ?l - location)
             (sampled ?r - resource)
             (free ?l - location)
             (deep ?l - location)
             (operational ?a - auv)
             (connected-ship ?s - ship ?l1 ?l2 - location)
             (isdeep ?a - auv)
             (onsurface ?a - auv)
             
)

(:action move-surface
:parameters (?a - auv ?l1 ?l2 - location)
:precondition (and (at ?a ?l1)
                   (operational ?a)
                   (onsurface ?a)
                   (connected ?l1 ?l2)
                   (free ?l2)
                
              )
:effect (and (not (at ?a ?l1))
             (at ?a ?l2)
             (free ?l1)
             (not (free ?l2))
        )
)

(:action move-deep
:parameters (?a - auv ?l1 ?l2 - location)
:precondition (and (at ?a ?l1)
                   (operational ?a)
                   (isdeep ?a)
                   (deep ?l2)
                   (connected ?l1 ?l2)
                   ;(free ?l2)
                
              )
:effect (and (not (at ?a ?l1))
             (at ?a ?l2)
             ;(free ?l1)
             ;(not (free ?l2))
        )
)

(:action descent
:parameters (?a - auv ?l - location)
:precondition (and (at ?a ?l)
                   (operational ?a)
                   (onsurface ?a)
                   (deep ?l)
                   
              )
:effect (and (not (onsurface ?a))
             (isdeep ?a)
             (free ?l)
        )
)

(:action ascent
:parameters (?a - auv ?l - location)
:precondition (and (at ?a ?l)
                   (operational ?a)
                   (isdeep ?a)
                   (free ?l)
                   
              )
:effect (and (not (isdeep ?a))
             (onsurface ?a)
             (not (free ?l))
        )
)

(:action sample
:parameters (?a - auv ?r - resource ?l - location)
:precondition (and (at ?a ?l)
                   (at-res ?r ?l)
                   (operational ?a)
              )
:effect (and (sampled ?r)
        )
)


)
