# Published by the dexterity node on /dexterity/events, e.g. for dexterity_evaluation
string TASK_STARTED = "task_started"
string PLANNING = "planning"
string TASK_FINISHED = "task_finished"

builtin_interfaces/Time stamp
string event

# task_started / task_finished: the task
string command                 # operation type, e.g. inspect_stationary
string tracker_name
string object_model
string object_side
float64 inspection_distance    # m
string gripper_frame_id
string result                  # task_finished: succeeded | cancelled | aborted: <reason>

# planning: one MoveIt plan of the controller
string controller_state        # interim | final | search
bool success
float64 planning_time          # s
float64 distance_to_object     # m, gripper -> point of interest (tracked pose), -1 if unknown
