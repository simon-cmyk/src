( DEFINE ( DOMAIN ROBPLAN )
( :REQUIREMENTS :ACTION-COSTS :CONDITIONAL-EFFECTS :TYPING )
( :TYPES
	VEHICLE - SUBJECT
	TURTLEBOT - VEHICLE
	ROBOT - VEHICLE
	CAMERA - VEHICLE
	CAMERA_EO - VEHICLE
	CAMERA_IR - VEHICLE
	ROBO_ARM - VEHICLE
	CHARGER - VEHICLE
	SUBJECT - OBJECT
	PHOTO - SUBJECT
	VALVE - SUBJECT
	PUMP - SUBJECT
	PIPE - SUBJECT
	SOUND - SUBJECT
	GAS_IND - SUBJECT
	OBJ - SUBJECT
	BATTERY - SUBJECT
	LOCATION - OBJECT
	CITY_LOCATION - LOCATION
	CITY - LOCATION
	WAYPOINT - CITY_LOCATION
	BATTERY_STATION - CITY_LOCATION
	ROUTE - OBJECT
	COUNTER - OBJECT
)
( :PREDICATES
	( END-PHASE )
	( STACK-PHASE )
	( START-PHASE )
	( AT ?SUBJECT0 - SUBJECT ?LOCATION1 - LOCATION )
	( PRE-AT ?SUBJECT0 - SUBJECT ?LOCATION1 - LOCATION )
	( LOCK-AT ?SUBJECT0 - SUBJECT ?LOCATION1 - LOCATION )
	( AVAILABLE ?VEHICLE0 - VEHICLE )
	( AVAILABLE ?CAMERA0 - CAMERA )
	( CONNECTS ?ROUTE0 - ROUTE ?LOCATION1 - LOCATION ?LOCATION2 - LOCATION )
	( IN_CITY ?LOCATION0 - LOCATION ?CITY1 - CITY )
	( ROUTE_AVAILABLE ?ROUTE0 - ROUTE )
	( NO_PHOTO ?SUBJECT0 - SUBJECT )
	( PHOTO ?SUBJECT0 - SUBJECT )
	( NO_SEALS_CHECK ?SUBJECT0 - SUBJECT )
	( PRE-NO_SEALS_CHECK ?SUBJECT0 - SUBJECT )
	( LOCK-NO_SEALS_CHECK ?SUBJECT0 - SUBJECT )
	( SEALS_CHECK ?SUBJECT0 - SUBJECT )
	( PRE-SEALS_CHECK ?SUBJECT0 - SUBJECT )
	( LOCK-SEALS_CHECK ?SUBJECT0 - SUBJECT )
	( ACTIVE0 )
	( STARTING0 )
	( ENDING0 )
	( ACTIVE1 )
	( STARTING1 )
	( ENDING1 )
	( ACTIVE2 )
	( STARTING2 )
	( ENDING2 )
	( ACTIVE-MOVE_ROBOT ?ROBOT0 - ROBOT ?LOCATION1 - LOCATION ?LOCATION2 - LOCATION ?ROUTE3 - ROUTE )
	( STARTING-MOVE_ROBOT ?ROBOT0 - ROBOT ?LOCATION1 - LOCATION ?LOCATION2 - LOCATION ?ROUTE3 - ROUTE )
	( ENDING-MOVE_ROBOT ?ROBOT0 - ROBOT ?LOCATION1 - LOCATION ?LOCATION2 - LOCATION ?ROUTE3 - ROUTE )
	( ACTIVE-CHECK_SEALS_VALVE_PICTURE_EO ?ROBOT0 - ROBOT ?LOCATION1 - LOCATION ?CAMERA_EO2 - CAMERA_EO ?VALVE3 - VALVE )
	( STARTING-CHECK_SEALS_VALVE_PICTURE_EO ?ROBOT0 - ROBOT ?LOCATION1 - LOCATION ?CAMERA_EO2 - CAMERA_EO ?VALVE3 - VALVE )
	( ENDING-CHECK_SEALS_VALVE_PICTURE_EO ?ROBOT0 - ROBOT ?LOCATION1 - LOCATION ?CAMERA_EO2 - CAMERA_EO ?VALVE3 - VALVE )
	( COUNT0-AT ?SUBJECT0 - SUBJECT ?LOCATION1 - LOCATION )
	( COUNT1-AT ?SUBJECT0 - SUBJECT ?LOCATION1 - LOCATION )
	( COUNT2-AT ?SUBJECT0 - SUBJECT ?LOCATION1 - LOCATION )
	( ENDING-COUNTER ?COUNTER0 - COUNTER )
	( CONSEC-COUNT ?COUNTER0 - COUNTER ?COUNTER1 - COUNTER )
	( STARTING-ALLOWED )
)
( :FUNCTIONS
	( TOTAL-COST )
	( MOVE_ROBOT-COST ?ROBOT0 - ROBOT ?ROUTE1 - ROUTE )
)
( :ACTION PUSH-MOVE_ROBOT
  :PARAMETERS ( ?ROBOT0 - ROBOT ?LOCATION1 - LOCATION ?LOCATION2 - LOCATION ?ROUTE3 - ROUTE )
  :PRECONDITION
	( AND
		( AT ?ROBOT0 ?LOCATION1 )
		( CONNECTS ?ROUTE3 ?LOCATION1 ?LOCATION2 )
		( STACK-PHASE )
		( NOT ( ACTIVE-MOVE_ROBOT ?ROBOT0 ?LOCATION1 ?LOCATION2 ?ROUTE3 ) )
		( NOT ( ACTIVE2 ) )
		( COUNT0-AT ?ROBOT0 ?LOCATION1 )
		( LOCK-AT ?ROBOT0 ?LOCATION1 )
		( PRE-AT ?ROBOT0 ?LOCATION1 )
		( LOCK-AT ?ROBOT0 ?LOCATION1 )
	)
  :EFFECT
	( AND
		( NOT ( AT ?ROBOT0 ?LOCATION1 ) )
		( STARTING-MOVE_ROBOT ?ROBOT0 ?LOCATION1 ?LOCATION2 ?ROUTE3 )
		( NOT ( PRE-AT ?ROBOT0 ?LOCATION1 ) )
		( NOT ( LOCK-AT ?ROBOT0 ?LOCATION1 ) )
		( WHEN
			( STARTING0 )
			( AND
				( NOT ( STARTING0 ) )
				( STARTING1 )
			)
		)
		( WHEN
			( ACTIVE0 )
			( AND
				( NOT ( ACTIVE0 ) )
				( ACTIVE1 )
			)
		)
		( WHEN
			( STARTING1 )
			( AND
				( NOT ( STARTING1 ) )
				( STARTING2 )
			)
		)
		( WHEN
			( ACTIVE1 )
			( AND
				( NOT ( ACTIVE1 ) )
				( ACTIVE2 )
			)
		)
		( INCREASE ( TOTAL-COST ) ( MOVE_ROBOT-COST ?ROBOT0 ?ROUTE3 ) )
		( STARTING-ALLOWED )
	)
)
( :ACTION LAUNCH-MOVE_ROBOT
  :PARAMETERS ( ?ROBOT0 - ROBOT ?LOCATION1 - LOCATION ?LOCATION2 - LOCATION ?ROUTE3 - ROUTE )
  :PRECONDITION
	( AND
		( START-PHASE )
		( STARTING-MOVE_ROBOT ?ROBOT0 ?LOCATION1 ?LOCATION2 ?ROUTE3 )
	)
  :EFFECT
	( AND
		( NOT ( STARTING-MOVE_ROBOT ?ROBOT0 ?LOCATION1 ?LOCATION2 ?ROUTE3 ) )
		( ACTIVE-MOVE_ROBOT ?ROBOT0 ?LOCATION1 ?LOCATION2 ?ROUTE3 )
		( WHEN
			( STARTING1 )
			( AND
				( NOT ( STARTING1 ) )
				( STARTING0 )
			)
		)
		( WHEN
			( STARTING2 )
			( AND
				( NOT ( STARTING2 ) )
				( STARTING1 )
			)
		)
		( INCREASE ( TOTAL-COST ) 1.000000 )
	)
)
( :ACTION POP-MOVE_ROBOT
  :PARAMETERS ( ?ROBOT0 - ROBOT ?LOCATION1 - LOCATION ?LOCATION2 - LOCATION ?ROUTE3 - ROUTE )
  :PRECONDITION
	( AND
		( END-PHASE )
		( ACTIVE-MOVE_ROBOT ?ROBOT0 ?LOCATION1 ?LOCATION2 ?ROUTE3 )
	)
  :EFFECT
	( AND
		( NOT ( ACTIVE-MOVE_ROBOT ?ROBOT0 ?LOCATION1 ?LOCATION2 ?ROUTE3 ) )
		( ENDING-MOVE_ROBOT ?ROBOT0 ?LOCATION1 ?LOCATION2 ?ROUTE3 )
		( WHEN
			( ACTIVE1 )
			( AND
				( NOT ( ACTIVE1 ) )
				( ACTIVE0 )
			)
		)
		( WHEN
			( ENDING0 )
			( AND
				( NOT ( ENDING0 ) )
				( ENDING1 )
			)
		)
		( WHEN
			( ACTIVE2 )
			( AND
				( NOT ( ACTIVE2 ) )
				( ACTIVE1 )
			)
		)
		( WHEN
			( ENDING1 )
			( AND
				( NOT ( ENDING1 ) )
				( ENDING2 )
			)
		)
		( INCREASE ( TOTAL-COST ) 1.000000 )
	)
)
( :ACTION FINISH-MOVE_ROBOT
  :PARAMETERS ( ?ROBOT0 - ROBOT ?LOCATION1 - LOCATION ?LOCATION2 - LOCATION ?ROUTE3 - ROUTE )
  :PRECONDITION
	( AND
		( STACK-PHASE )
		( ENDING-MOVE_ROBOT ?ROBOT0 ?LOCATION1 ?LOCATION2 ?ROUTE3 )
		( PRE-AT ?ROBOT0 ?LOCATION2 )
		( LOCK-AT ?ROBOT0 ?LOCATION2 )
	)
  :EFFECT
	( AND
		( AT ?ROBOT0 ?LOCATION2 )
		( NOT ( ENDING-MOVE_ROBOT ?ROBOT0 ?LOCATION1 ?LOCATION2 ?ROUTE3 ) )
		( NOT ( LOCK-AT ?ROBOT0 ?LOCATION2 ) )
		( WHEN
			( ENDING1 )
			( AND
				( NOT ( ENDING1 ) )
				( ENDING0 )
			)
		)
		( WHEN
			( ENDING2 )
			( AND
				( NOT ( ENDING2 ) )
				( ENDING1 )
			)
		)
		( STARTING-ALLOWED )
		( INCREASE ( TOTAL-COST ) 1.000000 )
	)
)
( :ACTION PUSH-CHECK_SEALS_VALVE_PICTURE_EO
  :PARAMETERS ( ?ROBOT0 - ROBOT ?LOCATION1 - LOCATION ?CAMERA_EO2 - CAMERA_EO ?VALVE3 - VALVE )
  :PRECONDITION
	( AND
		( AT ?VALVE3 ?LOCATION1 )
		( AVAILABLE ?CAMERA_EO2 )
		( NO_SEALS_CHECK ?VALVE3 )
		( STACK-PHASE )
		( NOT ( ACTIVE-CHECK_SEALS_VALVE_PICTURE_EO ?ROBOT0 ?LOCATION1 ?CAMERA_EO2 ?VALVE3 ) )
		( NOT ( ACTIVE2 ) )
		( LOCK-AT ?VALVE3 ?LOCATION1 )
		( LOCK-NO_SEALS_CHECK ?VALVE3 )
		( PRE-NO_SEALS_CHECK ?VALVE3 )
		( LOCK-NO_SEALS_CHECK ?VALVE3 )
	)
  :EFFECT
	( AND
		( NOT ( NO_SEALS_CHECK ?VALVE3 ) )
		( STARTING-CHECK_SEALS_VALVE_PICTURE_EO ?ROBOT0 ?LOCATION1 ?CAMERA_EO2 ?VALVE3 )
		( NOT ( PRE-AT ?VALVE3 ?LOCATION1 ) )
		( NOT ( PRE-NO_SEALS_CHECK ?VALVE3 ) )
		( NOT ( LOCK-NO_SEALS_CHECK ?VALVE3 ) )
		( WHEN
			( STARTING0 )
			( AND
				( NOT ( STARTING0 ) )
				( STARTING1 )
			)
		)
		( WHEN
			( ACTIVE0 )
			( AND
				( NOT ( ACTIVE0 ) )
				( ACTIVE1 )
			)
		)
		( WHEN
			( STARTING1 )
			( AND
				( NOT ( STARTING1 ) )
				( STARTING2 )
			)
		)
		( WHEN
			( ACTIVE1 )
			( AND
				( NOT ( ACTIVE1 ) )
				( ACTIVE2 )
			)
		)
		( INCREASE ( TOTAL-COST ) 100000.000000 )
		( STARTING-ALLOWED )
	)
)
( :ACTION LAUNCH-CHECK_SEALS_VALVE_PICTURE_EO
  :PARAMETERS ( ?ROBOT0 - ROBOT ?LOCATION1 - LOCATION ?CAMERA_EO2 - CAMERA_EO ?VALVE3 - VALVE )
  :PRECONDITION
	( AND
		( START-PHASE )
		( STARTING-CHECK_SEALS_VALVE_PICTURE_EO ?ROBOT0 ?LOCATION1 ?CAMERA_EO2 ?VALVE3 )
		( AT ?ROBOT0 ?LOCATION1 )
	)
  :EFFECT
	( AND
		( NOT ( STARTING-CHECK_SEALS_VALVE_PICTURE_EO ?ROBOT0 ?LOCATION1 ?CAMERA_EO2 ?VALVE3 ) )
		( ACTIVE-CHECK_SEALS_VALVE_PICTURE_EO ?ROBOT0 ?LOCATION1 ?CAMERA_EO2 ?VALVE3 )
		( WHEN
			( STARTING1 )
			( AND
				( NOT ( STARTING1 ) )
				( STARTING0 )
			)
		)
		( WHEN
			( STARTING2 )
			( AND
				( NOT ( STARTING2 ) )
				( STARTING1 )
			)
		)
		( WHEN
			( COUNT0-AT ?ROBOT0 ?LOCATION1 )
			( AND
				( NOT ( COUNT0-AT ?ROBOT0 ?LOCATION1 ) )
				( COUNT1-AT ?ROBOT0 ?LOCATION1 )
			)
		)
		( WHEN
			( COUNT1-AT ?ROBOT0 ?LOCATION1 )
			( AND
				( NOT ( COUNT1-AT ?ROBOT0 ?LOCATION1 ) )
				( COUNT2-AT ?ROBOT0 ?LOCATION1 )
			)
		)
		( INCREASE ( TOTAL-COST ) 1.000000 )
	)
)
( :ACTION POP-CHECK_SEALS_VALVE_PICTURE_EO
  :PARAMETERS ( ?ROBOT0 - ROBOT ?LOCATION1 - LOCATION ?CAMERA_EO2 - CAMERA_EO ?VALVE3 - VALVE )
  :PRECONDITION
	( AND
		( END-PHASE )
		( ACTIVE-CHECK_SEALS_VALVE_PICTURE_EO ?ROBOT0 ?LOCATION1 ?CAMERA_EO2 ?VALVE3 )
	)
  :EFFECT
	( AND
		( NOT ( ACTIVE-CHECK_SEALS_VALVE_PICTURE_EO ?ROBOT0 ?LOCATION1 ?CAMERA_EO2 ?VALVE3 ) )
		( ENDING-CHECK_SEALS_VALVE_PICTURE_EO ?ROBOT0 ?LOCATION1 ?CAMERA_EO2 ?VALVE3 )
		( WHEN
			( COUNT1-AT ?ROBOT0 ?LOCATION1 )
			( AND
				( NOT ( COUNT1-AT ?ROBOT0 ?LOCATION1 ) )
				( COUNT0-AT ?ROBOT0 ?LOCATION1 )
			)
		)
		( WHEN
			( COUNT2-AT ?ROBOT0 ?LOCATION1 )
			( AND
				( NOT ( COUNT2-AT ?ROBOT0 ?LOCATION1 ) )
				( COUNT1-AT ?ROBOT0 ?LOCATION1 )
			)
		)
		( WHEN
			( ACTIVE1 )
			( AND
				( NOT ( ACTIVE1 ) )
				( ACTIVE0 )
			)
		)
		( WHEN
			( ENDING0 )
			( AND
				( NOT ( ENDING0 ) )
				( ENDING1 )
			)
		)
		( WHEN
			( ACTIVE2 )
			( AND
				( NOT ( ACTIVE2 ) )
				( ACTIVE1 )
			)
		)
		( WHEN
			( ENDING1 )
			( AND
				( NOT ( ENDING1 ) )
				( ENDING2 )
			)
		)
		( INCREASE ( TOTAL-COST ) 1.000000 )
	)
)
( :ACTION FINISH-CHECK_SEALS_VALVE_PICTURE_EO
  :PARAMETERS ( ?ROBOT0 - ROBOT ?LOCATION1 - LOCATION ?CAMERA_EO2 - CAMERA_EO ?VALVE3 - VALVE )
  :PRECONDITION
	( AND
		( STACK-PHASE )
		( ENDING-CHECK_SEALS_VALVE_PICTURE_EO ?ROBOT0 ?LOCATION1 ?CAMERA_EO2 ?VALVE3 )
		( PRE-SEALS_CHECK ?VALVE3 )
		( LOCK-SEALS_CHECK ?VALVE3 )
	)
  :EFFECT
	( AND
		( SEALS_CHECK ?VALVE3 )
		( NOT ( ENDING-CHECK_SEALS_VALVE_PICTURE_EO ?ROBOT0 ?LOCATION1 ?CAMERA_EO2 ?VALVE3 ) )
		( NOT ( LOCK-SEALS_CHECK ?VALVE3 ) )
		( WHEN
			( ENDING1 )
			( AND
				( NOT ( ENDING1 ) )
				( ENDING0 )
			)
		)
		( WHEN
			( ENDING2 )
			( AND
				( NOT ( ENDING2 ) )
				( ENDING1 )
			)
		)
		( STARTING-ALLOWED )
		( INCREASE ( TOTAL-COST ) 1.000000 )
	)
)
( :ACTION ENDING-PHASE
  :PARAMETERS ( ?COUNTER0 - COUNTER ?COUNTER1 - COUNTER )
  :PRECONDITION
	( AND
		( START-PHASE )
		( STARTING0 )
		( ENDING-COUNTER ?COUNTER0 )
		( CONSEC-COUNT ?COUNTER0 ?COUNTER1 )
		( FORALL
			( ?SUBJECT2 - SUBJECT ?LOCATION3 - LOCATION )
			( AND
				( PRE-AT ?SUBJECT2 ?LOCATION3 )
				( LOCK-AT ?SUBJECT2 ?LOCATION3 )
			)
		)
		( FORALL
			( ?SUBJECT2 - SUBJECT )
			( AND
				( PRE-NO_SEALS_CHECK ?SUBJECT2 )
				( LOCK-NO_SEALS_CHECK ?SUBJECT2 )
			)
		)
		( FORALL
			( ?SUBJECT2 - SUBJECT )
			( AND
				( PRE-SEALS_CHECK ?SUBJECT2 )
				( LOCK-SEALS_CHECK ?SUBJECT2 )
			)
		)
	)
  :EFFECT
	( AND
		( NOT ( START-PHASE ) )
		( END-PHASE )
		( NOT ( ENDING-COUNTER ?COUNTER0 ) )
		( ENDING-COUNTER ?COUNTER1 )
		( INCREASE ( TOTAL-COST ) 1.000000 )
	)
)
( :ACTION STACKING-PHASE
  :PARAMETERS ( )
  :PRECONDITION
	( AND
		( END-PHASE )
	)
  :EFFECT
	( AND
		( NOT ( END-PHASE ) )
		( STACK-PHASE )
		( INCREASE ( TOTAL-COST ) 1.000000 )
	)
)
( :ACTION STARTING-PHASE
  :PARAMETERS ( )
  :PRECONDITION
	( AND
		( STACK-PHASE )
		( ENDING0 )
		( STARTING-ALLOWED )
	)
  :EFFECT
	( AND
		( NOT ( STACK-PHASE ) )
		( START-PHASE )
		( NOT ( STARTING-ALLOWED ) )
		( INCREASE ( TOTAL-COST ) 1.000000 )
	)
)
( :ACTION RESET-AT
  :PARAMETERS ( ?SUBJECT0 - SUBJECT ?LOCATION1 - LOCATION )
  :PRECONDITION
	( AND
		( START-PHASE )
	)
  :EFFECT
	( AND
		( PRE-AT ?SUBJECT0 ?LOCATION1 )
		( LOCK-AT ?SUBJECT0 ?LOCATION1 )
	)
)
( :ACTION RESET-NO_SEALS_CHECK
  :PARAMETERS ( ?SUBJECT0 - SUBJECT )
  :PRECONDITION
	( AND
		( START-PHASE )
	)
  :EFFECT
	( AND
		( PRE-NO_SEALS_CHECK ?SUBJECT0 )
		( LOCK-NO_SEALS_CHECK ?SUBJECT0 )
	)
)
( :ACTION RESET-SEALS_CHECK
  :PARAMETERS ( ?SUBJECT0 - SUBJECT )
  :PRECONDITION
	( AND
		( START-PHASE )
	)
  :EFFECT
	( AND
		( PRE-SEALS_CHECK ?SUBJECT0 )
		( LOCK-SEALS_CHECK ?SUBJECT0 )
	)
)
)