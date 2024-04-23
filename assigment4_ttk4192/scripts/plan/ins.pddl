( DEFINE ( PROBLEM ROBPLAN )
( :DOMAIN ROBPLAN )
( :OBJECTS
	TURTLEBOT0 - ROBOT
	CAMERA0 - CAMERA
	ROBO_ARM0 - ROBO_ARM
	VALVE0 VALVE1 - VALVE
	PUMP0 PUMP1 - PUMP
	WAYPOINT0 WAYPOINT1 WAYPOINT2 WAYPOINT3 WAYPOINT4 WAYPOINT5 WAYPOINT6 - WAYPOINT
	D01 D02 D03 D04 D05 D06 D10 D12 D13 D14 D15 D16 D20 D21 D23 D24 D25 D26 D30 D31 D32 D34 D35 D36 D40 D41 D42 D43 D45 D46 D50 D51 D52 D53 D54 D56 D60 D61 D62 D63 D64 D65 - ROUTE
	C0 C1 C2 C3 C4 C5 C6 C7 C8 C9 - COUNTER
)
( :INIT
	( CONNECTS D01 WAYPOINT0 WAYPOINT1 )
	( CONNECTS D02 WAYPOINT0 WAYPOINT2 )
	( CONNECTS D03 WAYPOINT0 WAYPOINT3 )
	( CONNECTS D04 WAYPOINT0 WAYPOINT4 )
	( CONNECTS D05 WAYPOINT0 WAYPOINT5 )
	( CONNECTS D06 WAYPOINT0 WAYPOINT6 )
	( CONNECTS D10 WAYPOINT1 WAYPOINT0 )
	( CONNECTS D12 WAYPOINT1 WAYPOINT2 )
	( CONNECTS D13 WAYPOINT1 WAYPOINT3 )
	( CONNECTS D14 WAYPOINT1 WAYPOINT4 )
	( CONNECTS D15 WAYPOINT1 WAYPOINT5 )
	( CONNECTS D16 WAYPOINT1 WAYPOINT6 )
	( CONNECTS D20 WAYPOINT2 WAYPOINT0 )
	( CONNECTS D21 WAYPOINT2 WAYPOINT1 )
	( CONNECTS D23 WAYPOINT2 WAYPOINT3 )
	( CONNECTS D24 WAYPOINT2 WAYPOINT4 )
	( CONNECTS D25 WAYPOINT2 WAYPOINT5 )
	( CONNECTS D26 WAYPOINT2 WAYPOINT6 )
	( CONNECTS D30 WAYPOINT3 WAYPOINT0 )
	( CONNECTS D31 WAYPOINT3 WAYPOINT1 )
	( CONNECTS D32 WAYPOINT3 WAYPOINT2 )
	( CONNECTS D34 WAYPOINT3 WAYPOINT4 )
	( CONNECTS D35 WAYPOINT3 WAYPOINT5 )
	( CONNECTS D36 WAYPOINT3 WAYPOINT6 )
	( CONNECTS D40 WAYPOINT4 WAYPOINT0 )
	( CONNECTS D41 WAYPOINT4 WAYPOINT1 )
	( CONNECTS D42 WAYPOINT4 WAYPOINT2 )
	( CONNECTS D43 WAYPOINT4 WAYPOINT3 )
	( CONNECTS D45 WAYPOINT4 WAYPOINT5 )
	( CONNECTS D46 WAYPOINT4 WAYPOINT6 )
	( CONNECTS D50 WAYPOINT5 WAYPOINT0 )
	( CONNECTS D51 WAYPOINT5 WAYPOINT1 )
	( CONNECTS D52 WAYPOINT5 WAYPOINT2 )
	( CONNECTS D53 WAYPOINT5 WAYPOINT3 )
	( CONNECTS D54 WAYPOINT5 WAYPOINT4 )
	( CONNECTS D56 WAYPOINT5 WAYPOINT6 )
	( CONNECTS D60 WAYPOINT6 WAYPOINT0 )
	( CONNECTS D61 WAYPOINT6 WAYPOINT1 )
	( CONNECTS D62 WAYPOINT6 WAYPOINT2 )
	( CONNECTS D63 WAYPOINT6 WAYPOINT3 )
	( CONNECTS D64 WAYPOINT6 WAYPOINT4 )
	( CONNECTS D65 WAYPOINT6 WAYPOINT5 )
	( AT TURTLEBOT0 WAYPOINT0 )
	( AT VALVE0 WAYPOINT1 )
	( AT VALVE1 WAYPOINT2 )
	( AT PUMP0 WAYPOINT5 )
	( AT PUMP1 WAYPOINT6 )
	( AVAILABLE TURTLEBOT0 )
	( AVAILABLE CAMERA0 )
	( AVAILABLE ROBO_ARM0 )
	( NO_SEALS_CHECK VALVE0 )
	( NO_SEALS_CHECK VALVE1 )
	( NO_SEALS_CHECK PUMP0 )
	( NO_SEALS_CHECK PUMP1 )
	( NO_VALVE_MANIPULATED VALVE0 )
	( ACTIVE0 )
	( STARTING0 )
	( ENDING0 )
	( START-PHASE )
	( ENDING-COUNTER C0 )
	( CONSEC-COUNT C0 C1 )
	( CONSEC-COUNT C1 C2 )
	( CONSEC-COUNT C2 C3 )
	( CONSEC-COUNT C3 C4 )
	( CONSEC-COUNT C4 C5 )
	( CONSEC-COUNT C5 C6 )
	( CONSEC-COUNT C6 C7 )
	( CONSEC-COUNT C7 C8 )
	( CONSEC-COUNT C8 C9 )
	( CONSEC-COUNT C9 C0 )
	( COUNT0-AT TURTLEBOT0 WAYPOINT0 )
	( COUNT0-AT TURTLEBOT0 WAYPOINT1 )
	( COUNT0-AT TURTLEBOT0 WAYPOINT2 )
	( COUNT0-AT TURTLEBOT0 WAYPOINT3 )
	( COUNT0-AT TURTLEBOT0 WAYPOINT4 )
	( COUNT0-AT TURTLEBOT0 WAYPOINT5 )
	( COUNT0-AT TURTLEBOT0 WAYPOINT6 )
	( COUNT0-AT CAMERA0 WAYPOINT0 )
	( COUNT0-AT CAMERA0 WAYPOINT1 )
	( COUNT0-AT CAMERA0 WAYPOINT2 )
	( COUNT0-AT CAMERA0 WAYPOINT3 )
	( COUNT0-AT CAMERA0 WAYPOINT4 )
	( COUNT0-AT CAMERA0 WAYPOINT5 )
	( COUNT0-AT CAMERA0 WAYPOINT6 )
	( COUNT0-AT ROBO_ARM0 WAYPOINT0 )
	( COUNT0-AT ROBO_ARM0 WAYPOINT1 )
	( COUNT0-AT ROBO_ARM0 WAYPOINT2 )
	( COUNT0-AT ROBO_ARM0 WAYPOINT3 )
	( COUNT0-AT ROBO_ARM0 WAYPOINT4 )
	( COUNT0-AT ROBO_ARM0 WAYPOINT5 )
	( COUNT0-AT ROBO_ARM0 WAYPOINT6 )
	( COUNT0-AT VALVE0 WAYPOINT0 )
	( COUNT0-AT VALVE0 WAYPOINT1 )
	( COUNT0-AT VALVE0 WAYPOINT2 )
	( COUNT0-AT VALVE0 WAYPOINT3 )
	( COUNT0-AT VALVE0 WAYPOINT4 )
	( COUNT0-AT VALVE0 WAYPOINT5 )
	( COUNT0-AT VALVE0 WAYPOINT6 )
	( COUNT0-AT VALVE1 WAYPOINT0 )
	( COUNT0-AT VALVE1 WAYPOINT1 )
	( COUNT0-AT VALVE1 WAYPOINT2 )
	( COUNT0-AT VALVE1 WAYPOINT3 )
	( COUNT0-AT VALVE1 WAYPOINT4 )
	( COUNT0-AT VALVE1 WAYPOINT5 )
	( COUNT0-AT VALVE1 WAYPOINT6 )
	( COUNT0-AT PUMP0 WAYPOINT0 )
	( COUNT0-AT PUMP0 WAYPOINT1 )
	( COUNT0-AT PUMP0 WAYPOINT2 )
	( COUNT0-AT PUMP0 WAYPOINT3 )
	( COUNT0-AT PUMP0 WAYPOINT4 )
	( COUNT0-AT PUMP0 WAYPOINT5 )
	( COUNT0-AT PUMP0 WAYPOINT6 )
	( COUNT0-AT PUMP1 WAYPOINT0 )
	( COUNT0-AT PUMP1 WAYPOINT1 )
	( COUNT0-AT PUMP1 WAYPOINT2 )
	( COUNT0-AT PUMP1 WAYPOINT3 )
	( COUNT0-AT PUMP1 WAYPOINT4 )
	( COUNT0-AT PUMP1 WAYPOINT5 )
	( COUNT0-AT PUMP1 WAYPOINT6 )
	( PRE-AT TURTLEBOT0 WAYPOINT0 )
	( PRE-AT TURTLEBOT0 WAYPOINT1 )
	( PRE-AT TURTLEBOT0 WAYPOINT2 )
	( PRE-AT TURTLEBOT0 WAYPOINT3 )
	( PRE-AT TURTLEBOT0 WAYPOINT4 )
	( PRE-AT TURTLEBOT0 WAYPOINT5 )
	( PRE-AT TURTLEBOT0 WAYPOINT6 )
	( PRE-AT CAMERA0 WAYPOINT0 )
	( PRE-AT CAMERA0 WAYPOINT1 )
	( PRE-AT CAMERA0 WAYPOINT2 )
	( PRE-AT CAMERA0 WAYPOINT3 )
	( PRE-AT CAMERA0 WAYPOINT4 )
	( PRE-AT CAMERA0 WAYPOINT5 )
	( PRE-AT CAMERA0 WAYPOINT6 )
	( PRE-AT ROBO_ARM0 WAYPOINT0 )
	( PRE-AT ROBO_ARM0 WAYPOINT1 )
	( PRE-AT ROBO_ARM0 WAYPOINT2 )
	( PRE-AT ROBO_ARM0 WAYPOINT3 )
	( PRE-AT ROBO_ARM0 WAYPOINT4 )
	( PRE-AT ROBO_ARM0 WAYPOINT5 )
	( PRE-AT ROBO_ARM0 WAYPOINT6 )
	( PRE-AT VALVE0 WAYPOINT0 )
	( PRE-AT VALVE0 WAYPOINT1 )
	( PRE-AT VALVE0 WAYPOINT2 )
	( PRE-AT VALVE0 WAYPOINT3 )
	( PRE-AT VALVE0 WAYPOINT4 )
	( PRE-AT VALVE0 WAYPOINT5 )
	( PRE-AT VALVE0 WAYPOINT6 )
	( PRE-AT VALVE1 WAYPOINT0 )
	( PRE-AT VALVE1 WAYPOINT1 )
	( PRE-AT VALVE1 WAYPOINT2 )
	( PRE-AT VALVE1 WAYPOINT3 )
	( PRE-AT VALVE1 WAYPOINT4 )
	( PRE-AT VALVE1 WAYPOINT5 )
	( PRE-AT VALVE1 WAYPOINT6 )
	( PRE-AT PUMP0 WAYPOINT0 )
	( PRE-AT PUMP0 WAYPOINT1 )
	( PRE-AT PUMP0 WAYPOINT2 )
	( PRE-AT PUMP0 WAYPOINT3 )
	( PRE-AT PUMP0 WAYPOINT4 )
	( PRE-AT PUMP0 WAYPOINT5 )
	( PRE-AT PUMP0 WAYPOINT6 )
	( PRE-AT PUMP1 WAYPOINT0 )
	( PRE-AT PUMP1 WAYPOINT1 )
	( PRE-AT PUMP1 WAYPOINT2 )
	( PRE-AT PUMP1 WAYPOINT3 )
	( PRE-AT PUMP1 WAYPOINT4 )
	( PRE-AT PUMP1 WAYPOINT5 )
	( PRE-AT PUMP1 WAYPOINT6 )
	( LOCK-AT TURTLEBOT0 WAYPOINT0 )
	( LOCK-AT TURTLEBOT0 WAYPOINT1 )
	( LOCK-AT TURTLEBOT0 WAYPOINT2 )
	( LOCK-AT TURTLEBOT0 WAYPOINT3 )
	( LOCK-AT TURTLEBOT0 WAYPOINT4 )
	( LOCK-AT TURTLEBOT0 WAYPOINT5 )
	( LOCK-AT TURTLEBOT0 WAYPOINT6 )
	( LOCK-AT CAMERA0 WAYPOINT0 )
	( LOCK-AT CAMERA0 WAYPOINT1 )
	( LOCK-AT CAMERA0 WAYPOINT2 )
	( LOCK-AT CAMERA0 WAYPOINT3 )
	( LOCK-AT CAMERA0 WAYPOINT4 )
	( LOCK-AT CAMERA0 WAYPOINT5 )
	( LOCK-AT CAMERA0 WAYPOINT6 )
	( LOCK-AT ROBO_ARM0 WAYPOINT0 )
	( LOCK-AT ROBO_ARM0 WAYPOINT1 )
	( LOCK-AT ROBO_ARM0 WAYPOINT2 )
	( LOCK-AT ROBO_ARM0 WAYPOINT3 )
	( LOCK-AT ROBO_ARM0 WAYPOINT4 )
	( LOCK-AT ROBO_ARM0 WAYPOINT5 )
	( LOCK-AT ROBO_ARM0 WAYPOINT6 )
	( LOCK-AT VALVE0 WAYPOINT0 )
	( LOCK-AT VALVE0 WAYPOINT1 )
	( LOCK-AT VALVE0 WAYPOINT2 )
	( LOCK-AT VALVE0 WAYPOINT3 )
	( LOCK-AT VALVE0 WAYPOINT4 )
	( LOCK-AT VALVE0 WAYPOINT5 )
	( LOCK-AT VALVE0 WAYPOINT6 )
	( LOCK-AT VALVE1 WAYPOINT0 )
	( LOCK-AT VALVE1 WAYPOINT1 )
	( LOCK-AT VALVE1 WAYPOINT2 )
	( LOCK-AT VALVE1 WAYPOINT3 )
	( LOCK-AT VALVE1 WAYPOINT4 )
	( LOCK-AT VALVE1 WAYPOINT5 )
	( LOCK-AT VALVE1 WAYPOINT6 )
	( LOCK-AT PUMP0 WAYPOINT0 )
	( LOCK-AT PUMP0 WAYPOINT1 )
	( LOCK-AT PUMP0 WAYPOINT2 )
	( LOCK-AT PUMP0 WAYPOINT3 )
	( LOCK-AT PUMP0 WAYPOINT4 )
	( LOCK-AT PUMP0 WAYPOINT5 )
	( LOCK-AT PUMP0 WAYPOINT6 )
	( LOCK-AT PUMP1 WAYPOINT0 )
	( LOCK-AT PUMP1 WAYPOINT1 )
	( LOCK-AT PUMP1 WAYPOINT2 )
	( LOCK-AT PUMP1 WAYPOINT3 )
	( LOCK-AT PUMP1 WAYPOINT4 )
	( LOCK-AT PUMP1 WAYPOINT5 )
	( LOCK-AT PUMP1 WAYPOINT6 )
	( PRE-NO_SEALS_CHECK TURTLEBOT0 )
	( PRE-NO_SEALS_CHECK CAMERA0 )
	( PRE-NO_SEALS_CHECK ROBO_ARM0 )
	( PRE-NO_SEALS_CHECK VALVE0 )
	( PRE-NO_SEALS_CHECK VALVE1 )
	( PRE-NO_SEALS_CHECK PUMP0 )
	( PRE-NO_SEALS_CHECK PUMP1 )
	( LOCK-NO_SEALS_CHECK TURTLEBOT0 )
	( LOCK-NO_SEALS_CHECK CAMERA0 )
	( LOCK-NO_SEALS_CHECK ROBO_ARM0 )
	( LOCK-NO_SEALS_CHECK VALVE0 )
	( LOCK-NO_SEALS_CHECK VALVE1 )
	( LOCK-NO_SEALS_CHECK PUMP0 )
	( LOCK-NO_SEALS_CHECK PUMP1 )
	( PRE-SEALS_CHECK TURTLEBOT0 )
	( PRE-SEALS_CHECK CAMERA0 )
	( PRE-SEALS_CHECK ROBO_ARM0 )
	( PRE-SEALS_CHECK VALVE0 )
	( PRE-SEALS_CHECK VALVE1 )
	( PRE-SEALS_CHECK PUMP0 )
	( PRE-SEALS_CHECK PUMP1 )
	( LOCK-SEALS_CHECK TURTLEBOT0 )
	( LOCK-SEALS_CHECK CAMERA0 )
	( LOCK-SEALS_CHECK ROBO_ARM0 )
	( LOCK-SEALS_CHECK VALVE0 )
	( LOCK-SEALS_CHECK VALVE1 )
	( LOCK-SEALS_CHECK PUMP0 )
	( LOCK-SEALS_CHECK PUMP1 )
	( PRE-NO_VALVE_MANIPULATED TURTLEBOT0 )
	( PRE-NO_VALVE_MANIPULATED CAMERA0 )
	( PRE-NO_VALVE_MANIPULATED ROBO_ARM0 )
	( PRE-NO_VALVE_MANIPULATED VALVE0 )
	( PRE-NO_VALVE_MANIPULATED VALVE1 )
	( PRE-NO_VALVE_MANIPULATED PUMP0 )
	( PRE-NO_VALVE_MANIPULATED PUMP1 )
	( LOCK-NO_VALVE_MANIPULATED TURTLEBOT0 )
	( LOCK-NO_VALVE_MANIPULATED CAMERA0 )
	( LOCK-NO_VALVE_MANIPULATED ROBO_ARM0 )
	( LOCK-NO_VALVE_MANIPULATED VALVE0 )
	( LOCK-NO_VALVE_MANIPULATED VALVE1 )
	( LOCK-NO_VALVE_MANIPULATED PUMP0 )
	( LOCK-NO_VALVE_MANIPULATED PUMP1 )
	( PRE-VALVE_MANIPULATED TURTLEBOT0 )
	( PRE-VALVE_MANIPULATED CAMERA0 )
	( PRE-VALVE_MANIPULATED ROBO_ARM0 )
	( PRE-VALVE_MANIPULATED VALVE0 )
	( PRE-VALVE_MANIPULATED VALVE1 )
	( PRE-VALVE_MANIPULATED PUMP0 )
	( PRE-VALVE_MANIPULATED PUMP1 )
	( LOCK-VALVE_MANIPULATED TURTLEBOT0 )
	( LOCK-VALVE_MANIPULATED CAMERA0 )
	( LOCK-VALVE_MANIPULATED ROBO_ARM0 )
	( LOCK-VALVE_MANIPULATED VALVE0 )
	( LOCK-VALVE_MANIPULATED VALVE1 )
	( LOCK-VALVE_MANIPULATED PUMP0 )
	( LOCK-VALVE_MANIPULATED PUMP1 )
	( = ( MOVE_ROBOT-COST TURTLEBOT0 D01 ) 83888 )
	( = ( MOVE_ROBOT-COST TURTLEBOT0 D02 ) 151666 )
	( = ( MOVE_ROBOT-COST TURTLEBOT0 D03 ) 203333 )
	( = ( MOVE_ROBOT-COST TURTLEBOT0 D04 ) 244444 )
	( = ( MOVE_ROBOT-COST TURTLEBOT0 D05 ) 127222 )
	( = ( MOVE_ROBOT-COST TURTLEBOT0 D06 ) 198888 )
	( = ( MOVE_ROBOT-COST TURTLEBOT0 D10 ) 83888 )
	( = ( MOVE_ROBOT-COST TURTLEBOT0 D12 ) 69444 )
	( = ( MOVE_ROBOT-COST TURTLEBOT0 D13 ) 140555 )
	( = ( MOVE_ROBOT-COST TURTLEBOT0 D14 ) 161111 )
	( = ( MOVE_ROBOT-COST TURTLEBOT0 D15 ) 123333 )
	( = ( MOVE_ROBOT-COST TURTLEBOT0 D16 ) 121666 )
	( = ( MOVE_ROBOT-COST TURTLEBOT0 D20 ) 151111 )
	( = ( MOVE_ROBOT-COST TURTLEBOT0 D21 ) 69444 )
	( = ( MOVE_ROBOT-COST TURTLEBOT0 D23 ) 92222 )
	( = ( MOVE_ROBOT-COST TURTLEBOT0 D24 ) 100555 )
	( = ( MOVE_ROBOT-COST TURTLEBOT0 D25 ) 140555 )
	( = ( MOVE_ROBOT-COST TURTLEBOT0 D26 ) 55000 )
	( = ( MOVE_ROBOT-COST TURTLEBOT0 D30 ) 203333 )
	( = ( MOVE_ROBOT-COST TURTLEBOT0 D31 ) 141111 )
	( = ( MOVE_ROBOT-COST TURTLEBOT0 D32 ) 92222 )
	( = ( MOVE_ROBOT-COST TURTLEBOT0 D34 ) 145000 )
	( = ( MOVE_ROBOT-COST TURTLEBOT0 D35 ) 122222 )
	( = ( MOVE_ROBOT-COST TURTLEBOT0 D36 ) 56111 )
	( = ( MOVE_ROBOT-COST TURTLEBOT0 D40 ) 244444 )
	( = ( MOVE_ROBOT-COST TURTLEBOT0 D41 ) 161111 )
	( = ( MOVE_ROBOT-COST TURTLEBOT0 D42 ) 100555 )
	( = ( MOVE_ROBOT-COST TURTLEBOT0 D43 ) 145000 )
	( = ( MOVE_ROBOT-COST TURTLEBOT0 D45 ) 236111 )
	( = ( MOVE_ROBOT-COST TURTLEBOT0 D46 ) 90555 )
	( = ( MOVE_ROBOT-COST TURTLEBOT0 D50 ) 127222 )
	( = ( MOVE_ROBOT-COST TURTLEBOT0 D51 ) 123333 )
	( = ( MOVE_ROBOT-COST TURTLEBOT0 D52 ) 140555 )
	( = ( MOVE_ROBOT-COST TURTLEBOT0 D53 ) 122222 )
	( = ( MOVE_ROBOT-COST TURTLEBOT0 D54 ) 236111 )
	( = ( MOVE_ROBOT-COST TURTLEBOT0 D56 ) 153888 )
	( = ( MOVE_ROBOT-COST TURTLEBOT0 D60 ) 198888 )
	( = ( MOVE_ROBOT-COST TURTLEBOT0 D61 ) 121666 )
	( = ( MOVE_ROBOT-COST TURTLEBOT0 D62 ) 55000 )
	( = ( MOVE_ROBOT-COST TURTLEBOT0 D63 ) 56111 )
	( = ( MOVE_ROBOT-COST TURTLEBOT0 D64 ) 90555 )
	( = ( MOVE_ROBOT-COST TURTLEBOT0 D65 ) 153888 )
	( = ( TOTAL-COST ) 0 )
)
( :GOAL
	( AND
		( AT TURTLEBOT0 WAYPOINT4 )
		( VALVE_MANIPULATED VALVE0 )
		( SEALS_CHECK PUMP0 )
		( SEALS_CHECK VALVE0 )
		( SEALS_CHECK VALVE1 )
		( SEALS_CHECK PUMP1 )
		( ACTIVE0 )
		( START-PHASE )
	)
)
( :METRIC MINIMIZE ( TOTAL-COST ) )
)