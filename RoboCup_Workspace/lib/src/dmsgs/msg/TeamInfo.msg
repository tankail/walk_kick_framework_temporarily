# general info
time txp_timestamp
time recv_timestamp
uint8 player_number
uint8 team_number
uint8 test
bool incapacitated

# Behavior info
uint8 role
bool attack_right

geometry_msgs/Vector3 dest
geometry_msgs/Vector3 final_dest
geometry_msgs/Vector3 attack_target
float32 time_since_last_kick

# playing state of robot
uint8 state
uint8 INACTIVE         = 0
uint8 PLAYING          = 1
uint8 BALL_HANDLING    = 2
uint8 PLACING          = 3
uint8 SEARCHING        = 4
uint8 INITIALIZING     = 5
uint8 ASSISTING        = 6

# teamwork info
bool priority
uint8 kicker_id
bool[6] mates_online
uint8 players_per_team
uint8[] online_player_numbers
uint8 active_agent_id
uint8 target_player_id
string proposed_tactic
string accepted_tactic
string coordination_position
string dynamic_role
string action_type
geometry_msgs/Vector3 ball_target_global
geometry_msgs/Vector3 target_speed
uint16 command_ttl_ms
bool strategy_mirror
string set_play_name
uint8 set_play_step

# Vision info
bool see_ball
bool see_circle
bool see_goal
geometry_msgs/Vector3 robot_pos
geometry_msgs/Vector3 ball_field
geometry_msgs/Vector3 ball_global
geometry_msgs/Vector3 ball_velocity

geometry_msgs/Vector3 circle_field
geometry_msgs/Vector3 circle_global

geometry_msgs/Vector3 goal_field
geometry_msgs/Vector3 goal_global

float32 ball_quality
float32 field_quality
float32 field_consistency

# GCInfo
bool gc_connected
uint8 gc_state
uint8 gamePhase
uint8 setPlay
uint8 players_per_team_gc
