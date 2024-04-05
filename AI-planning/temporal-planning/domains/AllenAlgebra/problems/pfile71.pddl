(define (problem aa-equal-4)
 (:domain allen-algebra)
 (:objects
	i1 i2 i3 i4 - interval
 )
 (:init
	(not-started i1)
	(not-ended i1)
	(not-started i2)
	(not-ended i2)
	(not-started i3)
	(not-ended i3)
	(not-started i4)
	(not-ended i4)
	(= (length i1) 5)
	(= (length i2) 5)
	(= (length i3) 10)
	(= (length i4) 10)
 )
 (:goal
	(and
		(equal i1 i2)
		(equal i3 i4)
	)
 )
)
