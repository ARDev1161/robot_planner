(define (domain mobile_robot_extended)
  (:requirements :typing :negative-preconditions :durative-actions :numeric-fluents :conditional-effects)

  (:types
    robot
    location
    object
    person
    target  ; цель для поиска (может быть человеком, другим роботом, докстанцией и т.п.)
  )

  (:predicates
    ;; Базовое положение робота
    (at ?r - robot ?l - location)

    ;; Предикат для отмечания посещённой локации
    (visited ?l - location)

    ;; Локация является частью патрульного маршрута
    (patrol_point ?l - location)

    ;; Предикат для информации о местонахождении человека
    (person_at ?p - person ?l - location)

    ;; Робот следует за человеком
    (following ?r - robot ?p - person)

    ;; Обнаружение цели (человека, другого робота, докстанции и т.п.)
    (found ?r - robot ?t - target)

    ;; Робот ведёт видеонаблюдение
    (video_active ?r - robot)
  )

  (:functions
    (cost)  ; суммарная стоимость выполнения плана (например, время, энергия и т.п.)
  )

  ;; Действие перемещения между локациями
  (:action move
    :parameters (?r - robot ?from - location ?to - location)
    :precondition (at ?r ?from)
    :effect (and
              (not (at ?r ?from))
              (at ?r ?to)
              (increase (cost) 1)
            )
  )

  ;; Действие посещения локации (отметить как посещённую)
  (:action visit
    :parameters (?r - robot ?l - location)
    :precondition (at ?r ?l)
    :effect (visited ?l)
  )

  ;; Действие патрулирования: переход от текущей точки к следующей (зацикленное перемещение)
  (:action patrol_move
    :parameters (?r - robot ?current - location ?next - location)
    :precondition (and (at ?r ?current) (patrol_point ?next))
    :effect (and
              (not (at ?r ?current))
              (at ?r ?next)
              (increase (cost) 1)
            )
  )

  ;; Действие следования за человеком
  (:action follow_person
    :parameters (?r - robot ?p - person ?current - location ?target - location)
    :precondition (and
                    (at ?r ?current)
                    (person_at ?p ?target)
                  )
    :effect (and
              (following ?r ?p)
              (at ?r ?target)  ; перемещаем робота в локацию, где находится человек
              (increase (cost) 1)
            )
  )

  ;; Действие поиска цели в текущей локации
  (:action search
    :parameters (?r - robot ?l - location ?t - target)
    :precondition (at ?r ?l)
    :effect (found ?r ?t)
  )

  ;; Действие ожидания (робот остаётся на месте)
  (:action wait
    :parameters (?r - robot ?l - location)
    :precondition (at ?r ?l)
    :effect (increase (cost) 1)
  )

  ;; Действие ожидания с видеонаблюдением
  (:action wait_and_watch
    :parameters (?r - robot ?l - location)
    :precondition (at ?r ?l)
    :effect (and
              (video_active ?r)
              (increase (cost) 1)
            )
  )
)
