(define (problem task)
(:domain transportationattendant)
(:objects
    rico - robot
    luke john - human
    glass sandwich - item
    initial - robot-location
    luke-location john-location wp0 wp1 - human-location
    glass-location sandwich-location spaghetti-location - item-location
)

(:init
    (at rico initial)
    (at john john-location)
    (at luke luke-location)

    (empty_robot)
    (not (not_empty_robot))

    (linked_to_location glass glass-location)
    (linked_to_location sandwich sandwich-location)
)

(:goal (and
    (load_left glass john wp0)
    (load_left sandwich luke wp1)
    )
)

)

