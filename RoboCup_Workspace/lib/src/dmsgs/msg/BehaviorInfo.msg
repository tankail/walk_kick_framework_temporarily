# Decision Making
uint8 current_role
string skill

# Destination
geometry_msgs/Vector3 dest
geometry_msgs/Vector3 final_dest
geometry_msgs/Vector3 attack_target

# Misc
bool save_image
bool more_lines
bool vision_correct_yaw

# ROLE
uint8 ROLE_STRIKER    = 1
uint8 ROLE_DEFENDER   = 2
uint8 ROLE_SUPPORTER  = 3
uint8 ROLE_GOALIE     = 4
uint8 ROLE_OTHER      = 5

bool attack_right

# Playing state of robot
uint8 team_play_state

# Priority of current behavior
uint8 kicker_id
bool team_play_priority

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

float32 time_since_last_kick
