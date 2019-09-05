(define (domain transportationattendant)

(:requirements :typing :adl :durative-actions :numeric-fluents)

(:types robot human - attendable
        attendable item - locatable
        greet-location human-location item-location robot-location waypoint - location)

(:predicates (empty_robot)
             (not_empty_robot)
             (item_on_robot ?item - item)
             (gave ?item - item ?human - human)
             (at ?obj - locatable ?loc - location)
             (attended ?obj - attendable ?from - location ?to - location)
             (linked_to_location ?lcbl - locatable ?loc - location))

(:durative-action GO
    :parameters (?obj - robot
                 ?start - location
                 ?destination - location)
    :duration (= ?duration 60)
    :condition (and
               (at start (at ?obj ?start))
               (over all (empty robot)))
    :effect (and
            (at end (at ?obj ?destination))
            (at start (not (at ?obj ?start))))
)

;(:durative-action GO_WITH_ATTENDANCE
;    :parameters (?obj - robot ?human - human ?start - location ?destination - location)
;    :duration (= ?duration 200)
;    :condition (and
;               (at start (at ?obj ?start))
;               (over all (not_empty_robot)))
;    :effect (and
;            (at end (at ?obj ?destination))
;            (at end (attended ?human ?start ?destination))
;            (at start (not (at ?obj ?start))))
;)
;
;(:durative-action GET_LOAD
;       :parameters (?obj - robot
;                    ?itemloc - item-location
;                    ?item - item)
;       :duration ( = ?duration 15)
;       :condition (and
;                  (at start (empty_robot))
;                  (over all (at ?obj ?itemloc))
;                  (over all (linked_to_location ?item ?itemloc)))
;        :effect (and
;                (at start (not_empty_robot))
;                (at start (not (empty_robot)))
;                (at end (item_on_robot ?item)))
;)
;
;(:durative-action LEAVE_LOAD
;       :parameters (?obj - robot
;                    ?item - item
;                    ?human - human
;                    ?destination - location)
;       :duration ( = ?duration 15)
;       :condition (and
;                  (at start (item_on_robot ?item))
;                  (at start (not_empty_robot))
;                  (over all (at ?obj ?destination))
;                  (over all (at ?human ?destination)))
;        :effect (and
;                (at end (not (not_empty_robot)))
;                (at end (empty_robot))
;                (at end (not (item_on_robot ?item)))
;                (at end (at ?item ?destination)))
;)


; TODO ACTION
;(:durative-action CHARGING_START
;      :parameters (?obj - robot ?humanloc - human-location ?human - human)
;      :duration ( = ?duration 30)
;      :condition (and
;                 (at start (human_coming))
;                 (over all (at ?obj ?humanloc))
;                 (over all (linked_to_location ?human ?humanloc)))
;      :effect (and
;              (at end (human_informed ?human)))
;)

)