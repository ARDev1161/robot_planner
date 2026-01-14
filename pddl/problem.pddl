(define (problem problemName)
  (:domain domainName)
  (:objects
    room_1 - room
    room_2 - room
    room_3 - room
    room_4 - room
    room_5 - room
    room_6 - room
    room_7 - room
    room_8 - room
    room_9 - room
    room_10 - room
    robot1 - robot
  )
  (:init
    ;; Связи между зонами (симметрично)
    (connected room_1 room_3)
    (connected room_3 room_1)
    (connected room_1 room_10)
    (connected room_10 room_1)
    (connected room_2 room_3)
    (connected room_3 room_2)
    (connected room_2 room_10)
    (connected room_10 room_2)
    (connected room_3 room_10)
    (connected room_10 room_3)
    (connected room_4 room_6)
    (connected room_6 room_4)
    (connected room_4 room_10)
    (connected room_10 room_4)
    (connected room_5 room_7)
    (connected room_7 room_5)
    (connected room_5 room_10)
    (connected room_10 room_5)
    (connected room_6 room_10)
    (connected room_10 room_6)
    (connected room_7 room_10)
    (connected room_10 room_7)
    (connected room_8 room_10)
    (connected room_10 room_8)
    (connected room_9 room_10)
    (connected room_10 room_9)

    ;; Начальное положение робота
    (robot_at robot1 room_1)
  )

  ;; Цель: переместить робота в room_10 (зону 10)
  (:goal (robot_at robot1 room_10))
)
