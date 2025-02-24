import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from tabulate import tabulate

# Suppress warning from matplotlib that too many plots are made and might consume memory
plt.rcParams.update({'figure.max_open_warning': 0})

# NOTE #
# User must specify simulation_runs
# This value must be set to the to the maximum iteration number
# that exists in the simluation data
simulation_runs = 10
DECIMALS = 2

# If this is active, only count the runs where the planner reached its goal and disregard the rest
# This is since the data from the runs where it didn't reach its goal means that it could have just stayed still for the entire run
# and hence gotten 0 collision, full survival time, low path length and low planning time, hence messing with the rest of the means.
REACHED_GOAL = False
EXPERIMENT_DATA_PATH = "/home/visualization/experiment_data_sdmp/mode8/"
modes = ["dynamic"] #"dynamic"

###################################### CONSTANTS ######################################

files = {
    'collision' : "collision.csv",
    'interval' : "intervals.csv",
    'logfile' : "logfile.csv",
    'summary' : "summary.csv",
    'goal' : "reached_goal.csv"
}

worlds = [
    "test",
    "cafe",
    "maze",
    "apartment",
    "tunnel",
    "field",
    "auditorium",
    "exhibition",
    "crosswalks",
    "patrol",
    "granso",
    "granso2"
]

scenarios = [
    "empty",
    "dynamic",
    "dynamic2",
    "dynamic3",
    "wall2",
    "wall",
    "wall3D",
    "moving_guard",
    "blind_corner",
    "corridor",
    "corridor2",
    "dead_end",
    "indoor1",
    "indoor11",
    "indoor2",
    "indoor3",
    "indoor_empty",
    "indoor_corner",
    "blocking_wall",
    "survival",
    "blind_corner_static",
    "dynamic2_static",
    "dynamic3_static",
    "indoor11_static",
    "survival_static",
    "corridor2_static",
]

# Subset of scenarios to test the different improvements
scenarios = [
            "survival"
            ]

tests = ["single",
         "full",
         "live",
         "eval-prim-exec",
         "random-goals"
]

planners = ["rrt_star",
            "time_based_rrt",
            "lattice_planner", 
            "lattice_planner_improved"
            ]

RRT_STAR = "rrt_star"
TIME_BASED_RRT = "time_based_rrt"
LATTICE_PLANNER = "lattice_planner"
LATTICE_PLANNER_IMPROVED = "lattice_planner_improved"

# Data tables
data_path = "visualized_data_sdmp/"
plots_path = "visualized_data_sdmp/plots/"
table_path = "visualized_data_sdmp/tables/"

collision_path = table_path + "collision.txt"
intervals = table_path + "intervals.txt"

paths = {
    RRT_STAR : table_path + "logfile_rrt_star.txt",
    TIME_BASED_RRT : table_path + "logfile_time_based_rrt.txt",
    LATTICE_PLANNER : table_path + "logfile_lp.txt",
    LATTICE_PLANNER_IMPROVED : table_path + "logfile_lp_improved.txt"
}

summary_paths = {
    RRT_STAR : table_path + "summary_rrt_star.txt",
    TIME_BASED_RRT : table_path + "summary_time_based_rrt.txt",
    LATTICE_PLANNER : table_path + "summary_lp.txt",
    LATTICE_PLANNER_IMPROVED : table_path + "summary_lp_improved.txt"
}


#################################### LABELS ###########################################################

def get_legend_name(df_key):
    if LATTICE_PLANNER_IMPROVED in df_key:
        return "LATTICE_PLANNER_IMPROVED"
    elif LATTICE_PLANNER in df_key:
        return "LATTICE_PLANNER"
    elif TIME_BASED_RRT in df_key:
        return "Temporal RRT*"
    elif RRT_STAR in df_key:
        return "RRT*"
    else:
        return "UNKNOWN"
    
#################################### TABLES ###########################################################

def visualize_logfile_lp():
    
    print("Creating Lattice Planner Logfile table...")

    logfile_headers = [
        "Motion Planner",
        "World",
        "Scenario",
        "Test Mode",
        "World Setting",
        "Planning Time Budget",
        "Replanning Time Budget",
        "Simulation Time (mean)",
        "Simulation Time (std)",
        "Search1 Time (mean)",
        "Search1 Time (std)",
        "Search2 Time (mean)",
        "Search2 Time (std)",
        "Total Search Time (mean)",
        "Total Search Time (std)",
        "Planning Cycle Time (mean)",
        "Planning Cycle Time (std)"
    ]
    
    logfile_df = pd.DataFrame(columns=logfile_headers)
    file = files["logfile"]
    
    for planner in [LATTICE_PLANNER]:
        for world in worlds:
            for mode in modes:
                for test in tests:
                    for scenario in scenarios:

                        planning_time = '-'
                        replanning_time = '-'
                        total_times = []
                        search1_times_means = []
                        search1_mean = '-'
                        search1_std = '-'
                        search2_times_means = []
                        search2_mean = '-'
                        search2_std = '-'
                        total_search_times_means = []
                        total_search_time_mean = '-'
                        total_search_time_std = '-'
                        planning_cycle_times_means = []
                        planning_cycle_time_mean = '-'
                        planning_cycle_time_std = '-'
                        at_least_one_iteration_exists = False

                        for iteration in range(simulation_runs):


                            # Logic to skip iteration if goal was not reached
                            reached_goal = True
                            if REACHED_GOAL: #Only count when it reached the goal
                                goal_reached_file_path = f"{EXPERIMENT_DATA_PATH}{planner}_{world}_{mode}_{test}_{scenario}_{iteration}/reached_goal.csv"
                                if os.path.isfile(goal_reached_file_path):
                                    df_goal = pd.read_csv(goal_reached_file_path)
                                    reached_goal = bool(df_goal["GoalReached"][0])

                            if not reached_goal:
                                continue

                            file_path = f"{EXPERIMENT_DATA_PATH}{planner}_{world}_{mode}_{test}_{scenario}_{iteration}/{file}"
                            if os.path.isfile(file_path):
                                #print("At least one iteration exist!")
                                at_least_one_iteration_exists = True

                                df = pd.read_csv(file_path)

                                planning_time = df[" Planning Time Budget [s]"].iloc[-1]
                                replanning_time = df[" Replanning Time Budget [s]"].iloc[-1]
                                total_times.append(df[" Simulation Time [s]"].iloc[-1])
                                search1_times_means.append(df[" Search1 Time [s]"].mean())
                                search2_times_means.append(df[" Search2 Time [s]"].mean())
                                total_search_times_means.append(df[" Total Search Time [s]"].mean())
                                planning_cycle_times_means.append(df[" Planning Cycle Time [s]"].mean())
                                
        
                        if at_least_one_iteration_exists:
                            
                            # Same for lattice_planner and time_based_rrt
                            total_times_mean = np.round(np.mean(total_times), DECIMALS)
                            total_times_std = np.round(np.std(total_times), DECIMALS)
                            search1_mean = np.round(np.mean(search1_times_means), DECIMALS)
                            search1_std = np.round(np.std(search1_times_means), DECIMALS)
                            search2_mean = np.round(np.mean(search2_times_means), DECIMALS)
                            search2_std = np.round(np.std(search2_times_means), DECIMALS)
                            total_search_time_mean = np.round(np.mean(total_search_times_means), DECIMALS)
                            total_search_time_std = np.round(np.std(total_search_times_means), DECIMALS)
                            planning_cycle_time_mean = np.round(np.mean(planning_cycle_times_means), DECIMALS)
                            planning_cycle_time_std = np.round(np.std(planning_cycle_times_means), DECIMALS)
                                

                            list_row = [planner, 
                                        world, 
                                        scenario, 
                                        test, 
                                        mode,
                                        planning_time, 
                                        replanning_time,
                                        total_times_mean, 
                                        total_times_std,
                                        search1_mean, 
                                        search1_std,
                                        search2_mean,
                                        search2_std,
                                        total_search_time_mean,
                                        total_search_time_std,
                                        planning_cycle_time_mean,
                                        planning_cycle_time_std]
                            
                            #print("Adding row", list_row)
                            logfile_df.loc[len(logfile_df)] = list_row

    table = tabulate(logfile_df, headers=logfile_headers, tablefmt="fancy_grid")

    with open(paths[planner], 'w') as f:
        f.write(table)

    return logfile_df


def visualize_logfile_lp_improved():
    
    print("Creating Lattice Planner Improved Logfile table...")

    logfile_headers = [
        "Motion Planner",
        "World",
        "Scenario",
        "Test Mode",
        "World Setting",
        "Planning Time Budget",
        "Replanning Time Budget",
        "Simulation Time (mean)",
        "Simulation Time (std)",
        "Search1 Time (mean)",
        "Search1 Time (std)",
        "Search2 Time (mean)",
        "Search2 Time (std)",
        "Total Search Time (mean)",
        "Total Search Time (std)",
        "Planning Cycle Time (mean)",
        "Planning Cycle Time (std)"
    ]
    
    logfile_df = pd.DataFrame(columns=logfile_headers)
    file = files["logfile"]
    
    for planner in [LATTICE_PLANNER_IMPROVED]:
        for world in worlds:
            for mode in modes:
                for test in tests:
                    for scenario in scenarios:

                        planning_time = '-'
                        replanning_time = '-'
                        total_times = []
                        search1_times_means = []
                        search1_mean = '-'
                        search1_std = '-'
                        search2_times_means = []
                        search2_mean = '-'
                        search2_std = '-'
                        total_search_times_means = []
                        total_search_time_mean = '-'
                        total_search_time_std = '-'
                        planning_cycle_times_means = []
                        planning_cycle_time_mean = '-'
                        planning_cycle_time_std = '-'
                        at_least_one_iteration_exists = False

                        for iteration in range(simulation_runs):

                            # Logic to skip iteration if goal was not reached
                            reached_goal = True
                            if REACHED_GOAL: #Only count when it reached the goal
                                goal_reached_file_path = f"{EXPERIMENT_DATA_PATH}{planner}_{world}_{mode}_{test}_{scenario}_{iteration}/reached_goal.csv"
                                if os.path.isfile(goal_reached_file_path):
                                    df_goal = pd.read_csv(goal_reached_file_path)
                                    reached_goal = bool(df_goal["GoalReached"][0])

                            if not reached_goal:
                                continue

                            file_path = f"{EXPERIMENT_DATA_PATH}{planner}_{world}_{mode}_{test}_{scenario}_{iteration}/{file}"
                            if os.path.isfile(file_path):
                                #print("At least one iteration exist!")
                                at_least_one_iteration_exists = True

                                df = pd.read_csv(file_path)

                                planning_time = df[" Planning Time Budget [s]"].iloc[-1]
                                replanning_time = df[" Replanning Time Budget [s]"].iloc[-1]
                                total_times.append(df[" Simulation Time [s]"].iloc[-1])
                                search1_times_means.append(df[" Search1 Time [s]"].mean())
                                search2_times_means.append(df[" Search2 Time [s]"].mean())
                                total_search_times_means.append(df[" Total Search Time [s]"].mean())
                                planning_cycle_times_means.append(df[" Planning Cycle Time [s]"].mean())
                                
        
                        if at_least_one_iteration_exists:
                            
                            # Same for lattice_planner/improved and time_based_rrt
                            total_times_mean = np.round(np.mean(total_times), DECIMALS)
                            total_times_std = np.round(np.std(total_times), DECIMALS)
                            search1_mean = np.round(np.mean(search1_times_means), DECIMALS)
                            search1_std = np.round(np.std(search1_times_means), DECIMALS)
                            search2_mean = np.round(np.mean(search2_times_means), DECIMALS)
                            search2_std = np.round(np.std(search2_times_means), DECIMALS)
                            total_search_time_mean = np.round(np.mean(total_search_times_means), DECIMALS)
                            total_search_time_std = np.round(np.std(total_search_times_means), DECIMALS)
                            planning_cycle_time_mean = np.round(np.mean(planning_cycle_times_means), DECIMALS)
                            planning_cycle_time_std = np.round(np.std(planning_cycle_times_means), DECIMALS)
                                

                            list_row = [planner, 
                                        world, 
                                        scenario, 
                                        test, 
                                        mode,
                                        planning_time, 
                                        replanning_time,
                                        total_times_mean, 
                                        total_times_std,
                                        search1_mean, 
                                        search1_std,
                                        search2_mean,
                                        search2_std,
                                        total_search_time_mean,
                                        total_search_time_std,
                                        planning_cycle_time_mean,
                                        planning_cycle_time_std]
                            
                            #print("Adding row", list_row)
                            logfile_df.loc[len(logfile_df)] = list_row

    table = tabulate(logfile_df, headers=logfile_headers, tablefmt="fancy_grid")

    with open(paths[planner], 'w') as f:
        f.write(table)

    return logfile_df


def visualize_collision(lp_improved_logfile, rrt_logfile, lp_logfile):

    print("Creating RRT*/ Time Based RRT* / (Improved) Lattice Planner Collision table...")


    # Create data frame and corresponding headers
    collision_headers = [
        "Motion Planner",
        "World",
        "Scenario",
        "Test Mode",
        "World Setting",
        "Number of Collision (mean)",
        "Number of Collision (std)",
        "Total Collision duration (mean)",
        "Total Collision duration (std)",
        "Average Collision Time",
        "Total number of collisions",
        "Number of dynamic collisions",
        "Number of static collisions",
        "Survival Time Mean (Time Until First collision)",
        "Survival Time Std"
    ]

    collision_df = pd.DataFrame(columns=collision_headers)
    file = files['collision']

    for planner in planners:
            for world in worlds:
                for mode in modes:
                    for test in tests:
                        for scenario in scenarios:

                            collision_mean = '-'
                            collision_std = '-'
                            total_duration_mean = '-'
                            total_duration_std = '-'
                            average_collision_time = '-'
                            total_nr_of_collisions = '-'
                            dynamic_collisions = '-'
                            static_collisions = '-'
                            survival_time_mean = 'Completed all runs without collisions'
                            survival_time_std = '-'

                            collisions = []
                            static = []
                            dynamic = []
                            total_duration_times = []
                            survival_times = []
                            at_least_one_iteration_exists = False

                            for iteration in range(simulation_runs):

                                # Logic to skip iteration if goal was not reached
                                reached_goal = True
                                if REACHED_GOAL: #Only count when it reached the goal
                                    goal_reached_file_path = f"{EXPERIMENT_DATA_PATH}{planner}_{world}_{mode}_{test}_{scenario}_{iteration}/reached_goal.csv"
                                    if os.path.isfile(goal_reached_file_path):
                                        df_goal = pd.read_csv(goal_reached_file_path)
                                        reached_goal = bool(df_goal["GoalReached"][0])

                                if not reached_goal:
                                    continue

                                file_path = f"{EXPERIMENT_DATA_PATH}{planner}_{world}_{mode}_{test}_{scenario}_{iteration}/{file}"
                                if os.path.isfile(file_path):
                                    at_least_one_iteration_exists = True

                                    #print(file_path) #Debug
                                    df = pd.read_csv(file_path)

                                    #Add the current number of collisions and time from each iteration to each list
                                    collisions.append(df["nr_of_collisions"][0])
                                    total_duration_times.append(df["Total duration"][0])
                                    dynamic.append(df["dynamic_collisions"][0])
                                    static.append(df["static_collisions"][0])
                            
                                interval_path = f"{EXPERIMENT_DATA_PATH}{planner}_{world}_{mode}_{test}_{scenario}_{iteration}/intervals.csv"
                                if os.path.isfile(interval_path):

                                    interval_df = pd.read_csv(interval_path)
                                    
                                    if(not interval_df.empty):
                                        survival_times.append(interval_df["start-time"][0])
                                    else:
                                        #Survived with no collisions, set survival time to average simulation time for specific scenario
                                        average_time = 0.0
                                        if planner == LATTICE_PLANNER_IMPROVED:
                                            average_time = lp_improved_logfile[(lp_improved_logfile["World"] == world) & (lp_improved_logfile["Scenario"] == scenario) & (lp_improved_logfile["Test Mode"] == test) & (lp_improved_logfile["World Setting"] == mode)]["Simulation Time (mean)"].iloc[0]
                                        elif planner == LATTICE_PLANNER:
                                            average_time = lp_logfile[(lp_logfile["World"] == world) & (lp_logfile["Scenario"] == scenario) & (lp_logfile["Test Mode"] == test) & (lp_logfile["World Setting"] == mode)]["Simulation Time (mean)"].iloc[0]
                                        elif planner == RRT_STAR or planner == TIME_BASED_RRT:
                                            average_time = rrt_logfile[(rrt_logfile["Motion Planner"] == planner) & (rrt_logfile["World"] == world) & (rrt_logfile["Scenario"] == scenario) & (rrt_logfile["Test Mode"] == test) & (rrt_logfile["World Setting"] == mode)]["Simulation Time (mean)"].iloc[0]
                                        else:
                                            print(f"Unknown planner {planner}")
                                        
                                        survival_times.append(average_time)

                            if at_least_one_iteration_exists:
                                collision_mean = np.mean(collisions)
                                collision_std = np.std(collisions)
                                total_duration_mean = np.mean(total_duration_times)
                                total_duration_std = np.std(total_duration_times)
                                total_nr_of_collisions = np.sum(collisions)
                                dynamic_collisions = np.sum(dynamic)
                                static_collisions = np.sum(static)
                                
                                if total_duration_mean != 0:
                                    average_collision_time = collision_mean/total_duration_mean
                                    
                                else:
                                    average_collision_time = 0

                                if len(survival_times) != 0:
                                    survival_time_mean = np.mean(survival_times)
                                    survival_time_std = np.std(survival_times)
                
                                # Create a row and insert at the end of the data frame
                                list_row = [planner, 
                                            world, 
                                            scenario, 
                                            test, 
                                            mode,
                                            collision_mean,
                                            collision_std,
                                            total_duration_mean,
                                            total_duration_std,
                                            average_collision_time,
                                            total_nr_of_collisions,
                                            dynamic_collisions,
                                            static_collisions,
                                            survival_time_mean,
                                            survival_time_std]
                                
                                collision_df.loc[len(collision_df)] = list_row

    
    ### Sort the data frame if needed by uncommenting and modifying the line below ###
    #collision_df = collision_df.sort_values(by=[collision_headers[3]], ascending=False)

    #Convert data frame to table
    table = tabulate(collision_df, headers=collision_headers, tablefmt="fancy_grid")

    with open(collision_path, 'w') as f:
        f.write(table)

    return collision_df

def visualize_logfile_rrt():

    print("Creating RRT* / Time Based RRT* Logfile table...")
    
    logfile_headers = [
        "Motion Planner",
        "World",
        "Scenario",
        "Test Mode",
        "World Setting",
        "Simulation Time (mean)",
        "Simulation Time (std)",
        "Search Time (mean)",
        "Search Time (std)",
        "Safe Path Time (mean)",
        "Safe Path Time (std)"
    ]
    
    file = files['logfile']

    # This will create only on df with both RRT and Time-based-RRT*. Two exact tables will be created.
    logfile_df = pd.DataFrame(columns=logfile_headers)
    
    for planner in [RRT_STAR, TIME_BASED_RRT]:
        for world in worlds:
            for mode in modes:
                for test in tests:
                    for scenario in scenarios:

                        total_times = []
                        total_times_mean = '-'
                        total_times_std = '-'
                
                        planning_cycle_times_means = []
                        planning_cycle_time_mean = '-'
                        planning_cycle_time_std = '-'
                
                        safe_path_time_means = []
                        safe_path_mean = '-'
                        safe_path_std = '-'

                        at_least_one_iteration_exists = False

                        for iteration in range(simulation_runs):

                            # Logic to skip iteration if goal was not reached
                            reached_goal = True
                            if REACHED_GOAL: #Only count when it reached the goal
                                goal_reached_file_path = f"{EXPERIMENT_DATA_PATH}{planner}_{world}_{mode}_{test}_{scenario}_{iteration}/reached_goal.csv"
                                if os.path.isfile(goal_reached_file_path):
                                    df_goal = pd.read_csv(goal_reached_file_path)
                                    reached_goal = bool(df_goal["GoalReached"][0])

                            if not reached_goal:
                                continue

                            file_path = f"{EXPERIMENT_DATA_PATH}{planner}_{world}_{mode}_{test}_{scenario}_{iteration}/{file}"
                            if os.path.isfile(file_path):

                                at_least_one_iteration_exists = True

                                df = pd.read_csv(file_path)

                                total_times.append(df[" Simulation Time [s]"].iloc[-1])
                                planning_cycle_times_means.append(df[" Total Search Time [s]"].mean())
                                safe_path_time_means.append(df[" Safe Path Time [s]"].mean())

                        if at_least_one_iteration_exists:
                            total_times_mean = np.round(np.mean(total_times), DECIMALS)
                            total_times_std = np.round(np.std(total_times), DECIMALS)                            
                            planning_cycle_time_mean = np.round(np.mean(planning_cycle_times_means), DECIMALS)
                            planning_cycle_time_std = np.round(np.std(planning_cycle_times_means), DECIMALS)
                            safe_path_mean = np.round(np.mean(safe_path_time_means), DECIMALS)
                            safe_path_std = np.round(np.std(safe_path_time_means), DECIMALS)

                            list_row = [planner, 
                                        world, 
                                        scenario, 
                                        test, 
                                        mode,
                                        total_times_mean, 
                                        total_times_std,
                                        planning_cycle_time_mean,
                                        planning_cycle_time_std,
                                        safe_path_mean,
                                        safe_path_std]
                            
                            logfile_df.loc[len(logfile_df)] = list_row

        table = tabulate(logfile_df, headers=logfile_headers, tablefmt="fancy_grid")

        with open(paths[planner], 'w') as f:
            f.write(table)
    
    return logfile_df


def visualize_summary_rrt():

    print("Creating RRT*/Time Based RRT* Summary table...")

    # Create data frame and corresponding headers
    summary_headers = [
        "Motion Planner",
        "World",
        "Scenario",
        "Test Mode",
        "World Setting",
        "acc_simulation_time_mean",
        "acc_simulation_time_std",
        "acc_planning_cycle_mean",
        "acc_planning_cycle_std",
        "acc_plansize_mean",
        "acc_plansize_std",
        "acc_eval_states_mean",
        "acc_eval_states_std",
        "acc_distance_mean",
        "acc_distance_std",
        "acc_safe_path_mean",
        "acc_safe_path_std",
        "reached goal?"
    ]

    file = files['summary']

    # This will create two identical tables further down with both RRT and TIME_BASED_RRT
    summary_df = pd.DataFrame(columns=summary_headers)

    for planner in [RRT_STAR, TIME_BASED_RRT]:
        for world in worlds:
            for mode in modes:
                for test in tests:
                    for scenario in scenarios:

                        acc_simulation_time_mean = '-'
                        acc_simulation_time_std = '-'
                        acc_planning_cycle_mean = '-'
                        acc_planning_cycle_std = '-'  
                        acc_plansize_mean = '-'
                        acc_plansize_std = '-'
                        acc_eval_states_mean = '-'
                        acc_eval_states_std = '-'
                        acc_distance_mean = '-'
                        acc_distance_std = '-'
                        acc_safe_path_mean = '-'
                        acc_safe_path_std = '-'

                        acc_simulation_time = []
                        acc_planning_cycle = []
                        acc_plansize = []
                        acc_eval_states = []
                        acc_distance = []
                        acc_safe_path = []
                        reached_goal = 0

                        at_least_one_iteration_exists = False

                        for iteration in range(simulation_runs):

                            # Logic to skip iteration if goal was not reached
                            reached_goal_flag = True
                            if REACHED_GOAL: #Only count when it reached the goal
                                goal_reached_file_path = f"{EXPERIMENT_DATA_PATH}{planner}_{world}_{mode}_{test}_{scenario}_{iteration}/reached_goal.csv"
                                if os.path.isfile(goal_reached_file_path):
                                    df_goal = pd.read_csv(goal_reached_file_path)
                                    reached_goal_flag = bool(df_goal["GoalReached"][0])

                            if not reached_goal_flag:
                                continue

                            file_path = f"{EXPERIMENT_DATA_PATH}{planner}_{world}_{mode}_{test}_{scenario}_{iteration}/{file}"
                            if os.path.isfile(file_path):
                                at_least_one_iteration_exists = True

                                df = pd.read_csv(file_path)

                                if(len(df[" Accumulated Plan Size [#State]"]) == 0):
                                    print("Summary-rrt: Empty summary")
                                    break
                                acc_simulation_time.append(df["Total Simulation Time [s]"].iloc[-1])
                                acc_planning_cycle.append(df[" Accumulated Total Search Time [s]"].iloc[-1])
                                acc_plansize.append(df[" Accumulated Plan Size [#State]"].iloc[-1])
                                acc_eval_states.append(df[" Accumulated Evaluated States [#State]"].iloc[-1])
                                acc_distance.append(df[" Accumulated Travelled Distance [m]"].iloc[-1])
                                acc_safe_path.append(df[" Accumulated Safe Path Time [s]"].iloc[-1])

                                goal_reached_file_path = f"{EXPERIMENT_DATA_PATH}{planner}_{world}_{mode}_{test}_{scenario}_{iteration}/reached_goal.csv"
                                if os.path.isfile(goal_reached_file_path):
                                    df_goal = pd.read_csv(goal_reached_file_path)
                                    
                                    reached_goal += df_goal["GoalReached"][0]

                        if at_least_one_iteration_exists:
                            acc_simulation_time_mean = np.round(np.mean(acc_simulation_time), DECIMALS)
                            acc_simulation_time_std = np.round(np.std(acc_simulation_time), DECIMALS)

                            acc_planning_cycle_mean = np.round(np.mean(acc_planning_cycle), DECIMALS)
                            acc_planning_cycle_std = np.round(np.std(acc_planning_cycle), DECIMALS)

                            acc_plansize_mean = np.round(np.mean(acc_plansize), DECIMALS)
                            acc_plansize_std = np.round(np.std(acc_plansize), DECIMALS)

                            acc_eval_states_mean = np.round(np.mean(acc_eval_states), DECIMALS)
                            acc_eval_states_std = np.round(np.std(acc_eval_states), DECIMALS)

                            acc_distance_mean = np.round(np.mean(acc_distance), DECIMALS)
                            acc_distance_std = np.round(np.std(acc_distance), DECIMALS)

                            acc_safe_path_mean = np.round(np.mean(acc_safe_path), DECIMALS)
                            acc_safe_path_std = np.round(np.std(acc_safe_path), DECIMALS)
            
                            # Create a row and insert at the end of the data frame
                            list_row = [planner,
                                        world,
                                        scenario,
                                        test,
                                        mode,
                                        acc_simulation_time_mean,
                                        acc_simulation_time_std,
                                        acc_planning_cycle_mean,
                                        acc_planning_cycle_std,
                                        acc_plansize_mean,
                                        acc_plansize_std,
                                        acc_eval_states_mean,
                                        acc_eval_states_std,
                                        acc_distance_mean,
                                        acc_distance_std,
                                        acc_safe_path_mean,
                                        acc_safe_path_std,
                                        reached_goal
                                ]
                            
                            summary_df.loc[len(summary_df)] = list_row


        ### Sort the data frame if needed by uncommenting and modifying the line below ###
        #summary_df = summary_df.sort_values(by=[collision_headers[3]], ascending=False)

        #Convert data frame to table
        table = tabulate(summary_df, headers=summary_headers, tablefmt="fancy_grid")

        with open(summary_paths[planner], 'w') as f:
            f.write(table)
    
    return summary_df


def visualize_summary_lp():

    print("Creating Lattice Planner Summary table...")
    
    # Create data frame and corresponding headers
    summary_headers = [
        "Motion Planner",
        "World",
        "Scenario",
        "Test Mode",
        "World Setting",
        "Acc Simulation Time (mean)",
        "Acc Simulation Time (std)",
        "acc s1 time (mean)",
        "acc s1 time (std)",
        "acc s2 time (mean)",
        "acc s2 time (std)",
        "acc total search (mean)",
        "acc total search (std)",
        "acc planning cycle (mean)",
        "acc planning cycle (std)",
        "acc openset (mean)",
        "acc openset (std)",
        "acc closedset (mean)",
        "acc closedset (std)",
        "acc plansize (mean)",
        "acc plansize (std)",
        "acc trajectory size (mean)",
        "acc trajectory size (std)",
        "acc travelled distance (mean)",
        "acc travelled distance (std)",
        "reached goal?"
    ]

    summary_df = pd.DataFrame(columns=summary_headers)
    file = files['summary']

    for planner in [LATTICE_PLANNER]:
            for world in worlds:
                for mode in modes:
                    for test in tests:
                        for scenario in scenarios:

                            acc_simulation_time_mean = '-'
                            acc_simulation_time_std = '-'
                            acc_s1_time_mean = '-'
                            acc_s1_time_std = '-'
                            acc_s2_time_mean = '-'
                            acc_s2_time_std = '-'
                            acc_total_search_mean = '-'
                            acc_total_search_std = '-'
                            acc_planning_cycle_mean = '-'
                            acc_planning_cycle_std = '-'
                            acc_openset_mean = '-'
                            acc_openset_std = '-'
                            acc_closedset_mean = '-'
                            acc_closedset_std = '-'
                            acc_plansize_mean = '-'
                            acc_plansize_std = '-'
                            acc_trajectory_size_mean = '-'
                            acc_trajectory_size_std = '-'
                            acc_distance_mean = '-'
                            acc_distance_std = '-'
                            reached_goal = 0


                            acc_simulation_time = []
                            acc_s1_time = []
                            acc_s2_time = []
                            acc_total_search = []
                            acc_planning_cycle = []
                            acc_openset = []
                            acc_closedset = []
                            acc_plansize = []
                            acc_trajectory_size = []
                            acc_distance = []

                            
                            at_least_one_iteration_exists = False

                            for iteration in range(simulation_runs):

                                # Logic to skip iteration if goal was not reached
                                reached_goal_flag = True
                                if REACHED_GOAL: #Only count when it reached the goal
                                    goal_reached_file_path = f"{EXPERIMENT_DATA_PATH}{planner}_{world}_{mode}_{test}_{scenario}_{iteration}/reached_goal.csv"
                                    if os.path.isfile(goal_reached_file_path):
                                        df_goal = pd.read_csv(goal_reached_file_path)
                                        reached_goal_flag = bool(df_goal["GoalReached"][0])

                                if not reached_goal_flag:
                                    continue

                                file_path = f"{EXPERIMENT_DATA_PATH}{planner}_{world}_{mode}_{test}_{scenario}_{iteration}/{file}"
                                if os.path.isfile(file_path):
                                    at_least_one_iteration_exists = True

                                    df = pd.read_csv(file_path)

                                    if ( len(df[" Accumulated Search1 Time [s]"]) == 0):
                                        print("Summary-lp: Empty summary")
                                        break

                                    acc_simulation_time.append(df["Total Simulation Time [s]"].iloc[-1])
                                    acc_s1_time.append(df[" Accumulated Search1 Time [s]"].iloc[-1])
                                    acc_s2_time.append(df[" Accumulated Search2 Time [s]"].iloc[-1])
                                    acc_total_search.append(df[" Accumulated Total Search Time [s]"].iloc[-1])
                                    acc_planning_cycle.append(df[" Accumulated Planning Cycle Time [s]"].iloc[-1])
                                    acc_openset.append(df[" Accumulated OpenSet [#State]"].iloc[-1])
                                    acc_closedset.append(df[" Accumulated ClosedSet [#State]"].iloc[-1])
                                    acc_plansize.append(df[" Accumulated Plan Size [#State]"].iloc[-1])
                                    acc_trajectory_size.append(df[" Accumulated Trajectory Plan Size [#TrajectoryState]"].iloc[-1])
                                    acc_distance.append(df[" Accumulated Travelled Distance [m]"].iloc[-1])
                                    
                                    goal_reached_file_path = f"{EXPERIMENT_DATA_PATH}{planner}_{world}_{mode}_{test}_{scenario}_{iteration}/reached_goal.csv"
                                    
                                    if os.path.isfile(goal_reached_file_path):
                                        df_goal = pd.read_csv(goal_reached_file_path)
                                        reached_goal += df_goal["GoalReached"][0]

                            if at_least_one_iteration_exists:
                                acc_simulation_time_mean = np.round(np.mean(acc_simulation_time), DECIMALS)
                                acc_simulation_time_std = np.round(np.std(acc_simulation_time), DECIMALS)

                                acc_s1_time_mean = np.round(np.mean(acc_s1_time), DECIMALS)
                                acc_s1_time_std = np.round(np.std(acc_s1_time), DECIMALS)

                                acc_s2_time_mean = np.round(np.mean(acc_s2_time), DECIMALS)
                                acc_s2_time_std = np.round(np.std(acc_s2_time), DECIMALS)

                                acc_total_search_mean = np.round(np.mean(acc_total_search), DECIMALS)
                                acc_total_search_std = np.round(np.std(acc_total_search), DECIMALS)

                                acc_planning_cycle_mean = np.round(np.mean(acc_planning_cycle), DECIMALS)
                                acc_planning_cycle_std = np.round(np.std(acc_planning_cycle), DECIMALS)

                                acc_openset_mean = np.round(np.mean(acc_openset), DECIMALS)
                                acc_openset_std = np.round(np.std(acc_openset), DECIMALS)

                                acc_closedset_mean = np.round(np.mean(acc_closedset), DECIMALS)
                                acc_closedset_std = np.round(np.std(acc_closedset), DECIMALS)

                                acc_plansize_mean = np.round(np.mean(acc_plansize), DECIMALS)
                                acc_plansize_std = np.round(np.std(acc_plansize), DECIMALS)

                                acc_trajectory_size_mean = np.round(np.mean(acc_trajectory_size), DECIMALS)
                                acc_trajectory_size_std = np.round(np.std(acc_trajectory_size), DECIMALS)

                                acc_distance_mean = np.round(np.mean(acc_distance), DECIMALS)
                                acc_distance_std = np.round(np.std(acc_distance), DECIMALS)
                                                
                                # Create a row and insert at the end of the data frame
                                list_row = [planner, 
                                            world, 
                                            scenario, 
                                            test, 
                                            mode,
                                            acc_simulation_time_mean,
                                            acc_simulation_time_std,
                                            acc_s1_time_mean,
                                            acc_s1_time_std,
                                            acc_s2_time_mean,
                                            acc_s2_time_std,
                                            acc_total_search_mean,
                                            acc_total_search_std,
                                            acc_planning_cycle_mean,
                                            acc_planning_cycle_std,
                                            acc_openset_mean,
                                            acc_openset_std,
                                            acc_closedset_mean,
                                            acc_closedset_std,
                                            acc_plansize_mean,
                                            acc_plansize_std,
                                            acc_trajectory_size_mean,
                                            acc_trajectory_size_std,
                                            acc_distance_mean,
                                            acc_distance_std,
                                            reached_goal
                                            ]
                                
                                summary_df.loc[len(summary_df)] = list_row

    
    ### Sort the data frame if needed by uncommenting and modifying the line below ###
    #summary_df = summary_df.sort_values(by=[collision_headers[3]], ascending=False)

    #Convert data frame to table
    table = tabulate(summary_df, headers=summary_headers, tablefmt="fancy_grid")

    with open(summary_paths[planner], 'w') as f:
        f.write(table)
    
    return summary_df


def visualize_summary_lp_improved():

    print("Creating Lattice Planner Improved Summary table...")
    
    # Create data frame and corresponding headers
    summary_headers = [
        "Motion Planner",
        "World",
        "Scenario",
        "Test Mode",
        "World Setting",
        "Acc Simulation Time (mean)",
        "Acc Simulation Time (std)",
        "acc s1 time (mean)",
        "acc s1 time (std)",
        "acc s2 time (mean)",
        "acc s2 time (std)",
        "acc total search (mean)",
        "acc total search (std)",
        "acc planning cycle (mean)",
        "acc planning cycle (std)",
        "acc openset (mean)",
        "acc openset (std)",
        "acc closedset (mean)",
        "acc closedset (std)",
        "acc plansize (mean)",
        "acc plansize (std)",
        "acc trajectory size (mean)",
        "acc trajectory size (std)",
        "acc travelled distance (mean)",
        "acc travelled distance (std)",
        "reached goal?"
    ]

    summary_df = pd.DataFrame(columns=summary_headers)
    file = files['summary']

    for planner in [LATTICE_PLANNER_IMPROVED]:
            for world in worlds:
                for mode in modes:
                    for test in tests:
                        for scenario in scenarios:

                            acc_simulation_time_mean = '-'
                            acc_simulation_time_std = '-'
                            acc_s1_time_mean = '-'
                            acc_s1_time_std = '-'
                            acc_s2_time_mean = '-'
                            acc_s2_time_std = '-'
                            acc_total_search_mean = '-'
                            acc_total_search_std = '-'
                            acc_planning_cycle_mean = '-'
                            acc_planning_cycle_std = '-'
                            acc_openset_mean = '-'
                            acc_openset_std = '-'
                            acc_closedset_mean = '-'
                            acc_closedset_std = '-'
                            acc_plansize_mean = '-'
                            acc_plansize_std = '-'
                            acc_trajectory_size_mean = '-'
                            acc_trajectory_size_std = '-'
                            acc_distance_mean = '-'
                            acc_distance_std = '-'
                            reached_goal = 0


                            acc_simulation_time = []
                            acc_s1_time = []
                            acc_s2_time = []
                            acc_total_search = []
                            acc_planning_cycle = []
                            acc_openset = []
                            acc_closedset = []
                            acc_plansize = []
                            acc_trajectory_size = []
                            acc_distance = []

                            
                            at_least_one_iteration_exists = False

                            for iteration in range(simulation_runs):

                                # Logic to skip iteration if goal was not reached
                                reached_goal_flag = True
                                if REACHED_GOAL: #Only count when it reached the goal
                                    goal_reached_file_path = f"{EXPERIMENT_DATA_PATH}{planner}_{world}_{mode}_{test}_{scenario}_{iteration}/reached_goal.csv"
                                    if os.path.isfile(goal_reached_file_path):
                                        df_goal = pd.read_csv(goal_reached_file_path)
                                        reached_goal_flag = bool(df_goal["GoalReached"][0])

                                if not reached_goal_flag:
                                    continue

                                file_path = f"{EXPERIMENT_DATA_PATH}{planner}_{world}_{mode}_{test}_{scenario}_{iteration}/{file}"
                                if os.path.isfile(file_path):
                                    at_least_one_iteration_exists = True

                                    df = pd.read_csv(file_path)

                                    if ( len(df[" Accumulated Search1 Time [s]"]) == 0):
                                        print("Summary-lp: Empty summary")
                                        break

                                    acc_simulation_time.append(df["Total Simulation Time [s]"].iloc[-1])
                                    acc_s1_time.append(df[" Accumulated Search1 Time [s]"].iloc[-1])
                                    acc_s2_time.append(df[" Accumulated Search2 Time [s]"].iloc[-1])
                                    acc_total_search.append(df[" Accumulated Total Search Time [s]"].iloc[-1])
                                    acc_planning_cycle.append(df[" Accumulated Planning Cycle Time [s]"].iloc[-1])
                                    acc_openset.append(df[" Accumulated OpenSet [#State]"].iloc[-1])
                                    acc_closedset.append(df[" Accumulated ClosedSet [#State]"].iloc[-1])
                                    acc_plansize.append(df[" Accumulated Plan Size [#State]"].iloc[-1])
                                    acc_trajectory_size.append(df[" Accumulated Trajectory Plan Size [#TrajectoryState]"].iloc[-1])
                                    acc_distance.append(df[" Accumulated Travelled Distance [m]"].iloc[-1])
                                    
                                    goal_reached_file_path = f"{EXPERIMENT_DATA_PATH}{planner}_{world}_{mode}_{test}_{scenario}_{iteration}/reached_goal.csv"
                                    
                                    if os.path.isfile(goal_reached_file_path):
                                        df_goal = pd.read_csv(goal_reached_file_path)
                                        reached_goal += df_goal["GoalReached"][0]

                            if at_least_one_iteration_exists:
                                acc_simulation_time_mean = np.round(np.mean(acc_simulation_time), DECIMALS)
                                acc_simulation_time_std = np.round(np.std(acc_simulation_time), DECIMALS)

                                acc_s1_time_mean = np.round(np.mean(acc_s1_time), DECIMALS)
                                acc_s1_time_std = np.round(np.std(acc_s1_time), DECIMALS)

                                acc_s2_time_mean = np.round(np.mean(acc_s2_time), DECIMALS)
                                acc_s2_time_std = np.round(np.std(acc_s2_time), DECIMALS)

                                acc_total_search_mean = np.round(np.mean(acc_total_search), DECIMALS)
                                acc_total_search_std = np.round(np.std(acc_total_search), DECIMALS)

                                acc_planning_cycle_mean = np.round(np.mean(acc_planning_cycle), DECIMALS)
                                acc_planning_cycle_std = np.round(np.std(acc_planning_cycle), DECIMALS)

                                acc_openset_mean = np.round(np.mean(acc_openset), DECIMALS)
                                acc_openset_std = np.round(np.std(acc_openset), DECIMALS)

                                acc_closedset_mean = np.round(np.mean(acc_closedset), DECIMALS)
                                acc_closedset_std = np.round(np.std(acc_closedset), DECIMALS)

                                acc_plansize_mean = np.round(np.mean(acc_plansize), DECIMALS)
                                acc_plansize_std = np.round(np.std(acc_plansize), DECIMALS)

                                acc_trajectory_size_mean = np.round(np.mean(acc_trajectory_size), DECIMALS)
                                acc_trajectory_size_std = np.round(np.std(acc_trajectory_size), DECIMALS)

                                acc_distance_mean = np.round(np.mean(acc_distance), DECIMALS)
                                acc_distance_std = np.round(np.std(acc_distance), DECIMALS)
                                                
                                # Create a row and insert at the end of the data frame
                                list_row = [planner, 
                                            world, 
                                            scenario, 
                                            test, 
                                            mode,
                                            acc_simulation_time_mean,
                                            acc_simulation_time_std,
                                            acc_s1_time_mean,
                                            acc_s1_time_std,
                                            acc_s2_time_mean,
                                            acc_s2_time_std,
                                            acc_total_search_mean,
                                            acc_total_search_std,
                                            acc_planning_cycle_mean,
                                            acc_planning_cycle_std,
                                            acc_openset_mean,
                                            acc_openset_std,
                                            acc_closedset_mean,
                                            acc_closedset_std,
                                            acc_plansize_mean,
                                            acc_plansize_std,
                                            acc_trajectory_size_mean,
                                            acc_trajectory_size_std,
                                            acc_distance_mean,
                                            acc_distance_std,
                                            reached_goal
                                            ]
                                
                                summary_df.loc[len(summary_df)] = list_row

    
    ### Sort the data frame if needed by uncommenting and modifying the line below ###
    #summary_df = summary_df.sort_values(by=[collision_headers[3]], ascending=False)

    #Convert data frame to table
    table = tabulate(summary_df, headers=summary_headers, tablefmt="fancy_grid")

    with open(summary_paths[planner], 'w') as f:
        f.write(table)

    return summary_df

#################################### PLOTS AVERAGE ###########################################################


def plot_metric_average(show):

    print("Creating Average Plot...")

    file = files['logfile']

    for metric in metrics:

        for scenario in scenarios:
            dfs = {}
            longest_time = None
            for planner in planners:
                for world in worlds:
                    for mode in modes:
                        for test in tests:
                                for iteration in range(simulation_runs):
                                    file_path = f"{EXPERIMENT_DATA_PATH}{planner}_{world}_{mode}_{test}_{scenario}_{iteration}/{file}"
                                    if os.path.isfile(file_path):
                                        df = pd.read_csv(file_path)
                                        
                                        if longest_time is None:
                                            longest_time = df[' Simulation Time [s]'].iloc[-1]
                                        
                                        elif df[' Simulation Time [s]'].iloc[-1] > longest_time:
                                            longest_time = df[' Simulation Time [s]'].iloc[-1]
                                        
                                        key = f"{scenario}_{planner}"
                                        if key in dfs:
                                            dfs[key].append(df)
                                        else:
                                            dfs[key] = [df]

            if longest_time is not None:   
                plot_average_metric(scenario, dfs, longest_time, metric)
        if show:
            plt.show()


def plot_average_metric(scenario, data_frames, longest_time, metric):
    
    N_interpolation_points = int(np.ceil(longest_time*2))

    plt.figure()

    for key in data_frames.keys():
        
        #Get all the simulation runs for a certain scenario and planner
        dfs = data_frames[key]
        N_df = len(dfs)
        Y = np.zeros((N_df, N_interpolation_points))
        max_t = 0
        
        for i in range(len(dfs)):
            if dfs[i][' Simulation Time [s]'].iloc[-1] > max_t:
                max_t = dfs[i][' Simulation Time [s]'].iloc[-1]

        color = get_color(key)

        for j, df in enumerate(dfs):

            y = get_df_col(df, key, metric)

            x = np.linspace(0, max_t, N_interpolation_points)
            interpolate_df = np.interp(x, np.linspace(0, max_t, len(y)), y)
            Y[j, :] = interpolate_df

        mean_Y = np.mean(Y, axis=0)
        std_Y = np.std(Y, axis=0)
        
        plt.plot(x, mean_Y, color=color, label=get_legend_name(key), linewidth=2)
        plt.plot(x, mean_Y + std_Y, color=color, linestyle='--', linewidth=2)
        plt.plot(x, mean_Y - std_Y, color=color, linestyle='--', linewidth=2)
        
        # Fill the area between the lines
        plt.fill_between(x, mean_Y - std_Y, mean_Y + std_Y, color=color, alpha=0.2)

    plt.xlabel('Simulation Time [s]')
    plt.ylabel(f'{metric} - {get_metric_unit(metric)}')
    plt.title(f'Scenario: {scenario} - Mean and Standard deviation')
        
    handles, labels = plt.gca().get_legend_handles_labels()
    
    if len(handles) >= 4:
        order = [0,1,2,3]
        plt.legend([handles[idx] for idx in order],[labels[idx] for idx in order])
    else:
        plt.legend()
            
    # create the subdirectory if it does not exist
    if not os.path.exists(plots_path + f"{scenario}/average"):
        os.makedirs(plots_path + f"{scenario}/average")
        
    # save the plot in the subdirectory
    plt.savefig(plots_path + f"{scenario}/average/{metric}_avg")


#################################### PLOTS HELPERS ###########################################################


metrics = ["planning_cycle", "cost_to_goal", "plan_size", "time_to_reach"]
lp_metrics = ["heuristic", "openset", "closedset", "trajectory"]
rrt_metrics = ["eval_states", "safe_path", "cost_to_goal_extra"]

def get_metric_unit(metric):
    if metric in ['planning_cycle',
                  'time_to_reach',
                  'safe_path'
                  ]:
        
        return '[s]'
    
    if metric in ['cost_to_goal',
                  'heuristic',
                  'cost_to_goal_extra'
                  ]:
        
        return '[m]'
    
    if metric in ['plan_size',
                  'openset',
                  'closedset',
                  'eval_states'
                  ]:
        
        return '[#State]'
    
    if metric == 'trajectory':
        return '[#TrajectoryState]'
    
    return None

def get_metric_value(metric, planner, df):

    if (metric not in metrics) and (metric not in lp_metrics) and (metric not in rrt_metrics):
        print("Unknown metric!")
        return None

    if metric == 'planning_cycle':
        if planner == LATTICE_PLANNER_IMPROVED or planner == LATTICE_PLANNER:
            return df[' Planning Cycle Time [s]'].sum()
        else:
            return df[' Total Search Time [s]'].sum()
    
    if metric == 'cost_to_goal':
        if planner == LATTICE_PLANNER_IMPROVED or planner == LATTICE_PLANNER:
            return df[' Cost Start to Goal [m]'].sum()
        else:
            return df[' Cost to Goal [m]'].sum()

    if metric == 'plan_size':
        if planner == LATTICE_PLANNER_IMPROVED or planner == LATTICE_PLANNER:
            return df[' Plan Size [#State]'].sum()
        else:
            return df[' Plan Size [#State]'].sum()

    if metric == 'time_to_reach':
        if planner == LATTICE_PLANNER_IMPROVED or planner == LATTICE_PLANNER:
            return df[" Time to reach Goal [s]"].sum()
        else:
            return df[' Time to reach Goal [s]'].sum()
        
    if metric == "heuristic":
        return df[' Heuristic Estimate to Goal [m]'].sum()
    
    if metric == 'openset':
        return df[' OpenSet [#State]'].sum()
    
    if metric == 'closedset':
        return df[' ClosedSet [#State]'].sum()

    if metric == 'trajectory':
        return df[' Trajectory Plan Size [#TrajectoryState]'].sum()

    if metric == "eval_states":
        return df[' Evaluated States [#State]'].sum()
    
    if metric == "safe_path":
        return df[' Safe Path Time [s]'].sum()

    if metric == "cost_to_goal_extra":
        return df[' Cost to Goal [m]'].sum()


def compare_metric(metric, res, best):

    # Favor smaller values
    if metric in ["planning_cycle", 
                  "cost_to_goal", 
                  "plan_size", 
                  "time_to_reach", 
                  'heuristic', 
                  'heuristic', 
                  'closedset', 
                  'trajectory', 
                  "eval_states", 
                  "safe_path", 
                  "cost_to_goal_extra"]:
        
        return res < best

    # Favor larget values
    if metric in [""]:
        return res > best

def get_df_col(df, df_key, metric):
     
    if (metric not in metrics) and (metric not in lp_metrics) and (metric not in rrt_metrics):
        print("Unknown metric!")
        return None

    if metric == 'planning_cycle':
        if LATTICE_PLANNER_IMPROVED in df_key or LATTICE_PLANNER in df_key:
            return df[' Planning Cycle Time [s]']
        else:
            return df[' Total Search Time [s]']
    
    if metric == 'cost_to_goal':
        if LATTICE_PLANNER_IMPROVED in df_key or LATTICE_PLANNER in df_key:
            return df[' Cost Start to Goal [m]']
        else:
            return df[' Cost to Goal [m]']

    if metric == 'plan_size':
        if LATTICE_PLANNER_IMPROVED in df_key or LATTICE_PLANNER in df_key:
            return df[' Plan Size [#State]']
        else:
            return df[' Plan Size [#State]']

    if metric == 'time_to_reach':
        if LATTICE_PLANNER_IMPROVED in df_key or LATTICE_PLANNER in df_key:
            return df[" Time to reach Goal [s]"]
        else:
            return df[' Time to reach Goal [s]']
        
    if metric == "heuristic":
        return df[' Heuristic Estimate to Goal [m]']
    
    if metric == 'openset':
        return df[' OpenSet [#State]']
    
    if metric == 'closedset':
        return df[' ClosedSet [#State]']

    if metric == 'trajectory':
        return df[' Trajectory Plan Size [#TrajectoryState]']

    if metric == "eval_states":
        return df[' Evaluated States [#State]']
    
    if metric == "safe_path":
        return df[' Safe Path Time [s]']

    if metric == "cost_to_goal_extra":
        return df[' Cost to Goal [m]']


def get_key(data_frames, df_key, metric):

    if (metric not in metrics) and (metric not in lp_metrics) and (metric not in rrt_metrics):
        print("ERROR: UNKNOWN METRIC!")
        return 0
    
            
    if metric == "planning_cycle":
        if LATTICE_PLANNER in df_key:
            return data_frames[df_key].columns[7]
        else:
            return data_frames[df_key].columns[2]

    if metric == "cost_to_goal":
        if LATTICE_PLANNER in df_key:
            return data_frames[df_key].columns[9]
        else:
            return data_frames[df_key].columns[3]

    if metric == "plan_size":
        if LATTICE_PLANNER in df_key:
            return data_frames[df_key].columns[12]
        else:
            return data_frames[df_key].columns[5]

    if metric == "time_to_reach":
        if LATTICE_PLANNER in df_key:
            return data_frames[df_key].columns[14]
        else:
            return data_frames[df_key].columns[4]
        
    if metric == "heuristic":
        return data_frames[df_key].columns[12]
    
    if metric == 'openset':
        return data_frames[df_key].columns[10]
    
    if metric == 'closedset':
        return data_frames[df_key].columns[11]

    if metric == 'trajectory':
        return data_frames[df_key].columns[13]

    if metric == "eval_states":
        return data_frames[df_key].columns[6]
    
    if metric == "safe_path":
        return data_frames[df_key].columns[10]

    if metric == "cost_to_goal_extra":
        return data_frames[df_key].columns[4]


def show_collisions(metric):
    show_collisions = ["planning_cycle", "cost_to_goal"]
    return metric in show_collisions


def get_color(df_key):
    color = ''

    if LATTICE_PLANNER_IMPROVED in df_key:
        color = 'cyan'
    elif LATTICE_PLANNER in df_key:
        color = 'olive'
    elif TIME_BASED_RRT in df_key:
        color = 'gray'
    elif RRT_STAR in df_key:
        color = 'pink'
    else:
        print("Planner does not exist in color palette!")
        color = "green"  

    return color

#################################### PLOTS BEST RUN ###########################################################


def plot_common_metrics(show):

    print("Creating common metrics plots...")

    file = files['logfile']
    for metric in metrics:

        for scenario in scenarios:
            best_dfs = {}
            interval_dfs = {}
            reached_goal_dfs = {}
            for planner in planners:
                best = 0
                best_df = None
                best_simulation = 0
                best_path = ""
                for world in worlds:
                    for mode in modes:
                        for test in tests:
                                for iteration in range(simulation_runs):
                                    file_path = f"{EXPERIMENT_DATA_PATH}{planner}_{world}_{mode}_{test}_{scenario}_{iteration}/{file}"
                                    if os.path.isfile(file_path):
                                        df = pd.read_csv(file_path)

                                        res = get_metric_value(metric, planner, df)

                                        reached_goal_path = f"{EXPERIMENT_DATA_PATH}{planner}_{world}_{mode}_{test}_{scenario}_{iteration}/reached_goal.csv"

                                        if os.path.isfile(reached_goal_path):
                                            df_goal = pd.read_csv(reached_goal_path)
                                            reached_goal = df_goal["GoalReached"][0]
                                        else:
                                            print("Reached Goal does not exists, abort...")
                                            return

                                        if best_df is None and reached_goal:
                                            best_df = df
                                            best_simulation = iteration
                                            best = res
                                            best_path = f"{EXPERIMENT_DATA_PATH}{planner}_{world}_{mode}_{test}_{scenario}_{best_simulation}"

                                        elif compare_metric(metric, res, best) and reached_goal:
                                            best = res
                                            best_df = df
                                            best_simulation = iteration
                                            best_path = f"{EXPERIMENT_DATA_PATH}{planner}_{world}_{mode}_{test}_{scenario}_{best_simulation}"                                      

                if best_df is not None:
                    best_simulation_path = best_path + "/intervals.csv"
                    goal_reached_file_path = best_path + "/reached_goal.csv"

                    key = f"{scenario}_{planner}"
                    best_dfs[key] = best_df
                    interval_dfs[key] = pd.read_csv(best_simulation_path)
                    reached_goal_dfs[key] = pd.read_csv(goal_reached_file_path)

            if len(best_dfs) != 0:
                plot_metric(scenario, best_dfs, interval_dfs, metric, reached_goal_dfs)
        
        if show:
            plt.show()


def plot_metric(scenario, data_frames, interval_dfs, metric, reached_goal_dfs = {}):
    
    plt.figure()
    paintOnce = True

    for i, df_key in enumerate(data_frames):

        color = get_color(df_key) 

        # Cycle data
        key = get_key(data_frames, df_key, metric)
        data_col = data_frames[df_key][key]
        time_col = data_frames[df_key][' Simulation Time [s]']

        # Collision data
        start_col = interval_dfs[df_key]["start-time"]
        end_col = interval_dfs[df_key]["end-time"]

        # Reached goal data
        reached_goal = False
        if reached_goal_dfs:
            reached_goal = reached_goal_dfs[df_key]["GoalReached"][0]
        
        plt.plot(time_col, data_col, label=get_legend_name(df_key),linestyle='-', color=color)

        if reached_goal:
            plt.plot(time_col.iloc[-1], data_col.iloc[-1], 'x', color="red")

        ## Uncomment these lines to show crosses at the start and end of collision for each planner in the plot ##
        #crosses_start = np.interp(start_col, time_col, data_col)
        #crosses_end = np.interp(end_col, time_col, data_col)
        #plt.plot(start_col, crosses_start, 'x', color="red")        
        #plt.plot(start_col, crosses_end, 'x', color="green") 

        if show_collisions(metric):
            for j in range(len(start_col)):
                duration = np.linspace(start_col[j], end_col[j], 500)
                duration_interpol = np.interp(duration, time_col, data_col)
                if(paintOnce):
                    plt.plot(duration, duration_interpol, color="black", linewidth=5, label="Collision-segment")
                    paintOnce = False
                else:
                    plt.plot(duration, duration_interpol, color="black", linewidth=5)


    plt.title(f"Scenario: {scenario} - {metric}")
    plt.xlabel('Time [s]')
    plt.ylabel(f'{metric} - {get_metric_unit(metric)}')
    
    handles, labels = plt.gca().get_legend_handles_labels()
    if len(labels) > len(planners):
        order = [1,0,2,3,4]
        plt.legend([handles[idx] for idx in order],[labels[idx] for idx in order])
    else:
        plt.legend()

    sub_folder = ''
    if metric in metrics:
        sub_folder = plots_path + f"{scenario}/common"
    elif metric in lp_metrics:
        sub_folder = plots_path + f"{scenario}/lp_metrics"
    elif metric in rrt_metrics:
        sub_folder = plots_path + f"{scenario}/rrt_metrics"
    else:
        print(f"Unknown metric, setting to default path {sub_folder}")

    # create the subdirectory if it does not exist
    if not os.path.exists(sub_folder):
        os.makedirs(sub_folder)
        
    # save the plot in the subdirectory
    plt.savefig(sub_folder + f"/{metric}")


def plot_lattice_planner_metrics(show):

    print("Creating Lattice Planner (+ Improved) Metrics Plot...")
    
    file = files['logfile']
    for metric in lp_metrics:

        for scenario in scenarios:
            best_dfs = {}
            interval_dfs = {}
            reached_goal_dfs = {}
            for planner in [LATTICE_PLANNER, LATTICE_PLANNER_IMPROVED]:
                best = 0
                best_df = None
                best_simulation = 0
                best_path = ""
                for world in worlds:
                    for mode in modes:
                        for test in tests:
                                for iteration in range(simulation_runs):
                                    file_path = f"{EXPERIMENT_DATA_PATH}{planner}_{world}_{mode}_{test}_{scenario}_{iteration}/{file}"
                                    if os.path.isfile(file_path):
                                        df = pd.read_csv(file_path)

                                        res = get_metric_value(metric, planner, df)

                                        reached_goal_path = f"{EXPERIMENT_DATA_PATH}{planner}_{world}_{mode}_{test}_{scenario}_{iteration}/reached_goal.csv"

                                        if os.path.isfile(reached_goal_path):
                                            df_goal = pd.read_csv(reached_goal_path)
                                            reached_goal = df_goal["GoalReached"][0]
                                        else:
                                            print("Reached Goal does not exists, abort...")
                                            return

                                        if best_df is None and reached_goal:
                                            best_df = df
                                            best_simulation = iteration
                                            best = res
                                            best_path = f"{EXPERIMENT_DATA_PATH}{planner}_{world}_{mode}_{test}_{scenario}_{best_simulation}"

                                        elif compare_metric(metric, res, best) and reached_goal:
                                            best = res
                                            best_df = df
                                            best_simulation = iteration
                                            best_path = f"{EXPERIMENT_DATA_PATH}{planner}_{world}_{mode}_{test}_{scenario}_{best_simulation}"

                if best_df is not None:
                    best_simulation_path = best_path + "/intervals.csv"
                    goal_reached_file_path = best_path + "/reached_goal.csv"

                    key = f"{scenario}_{planner}"
                    best_dfs[key] = best_df
                    interval_dfs[key] = pd.read_csv(best_simulation_path)
                    reached_goal_dfs[key] = pd.read_csv(goal_reached_file_path)


            if len(best_dfs) != 0:
                plot_metric(scenario, best_dfs, interval_dfs, metric, reached_goal_dfs)
        
        if show:
            plt.show()


def plot_rrt_metrics(show):

    print("Creating RRT* / Time Based RRT* Metrics Plot...")
        
    file = files['logfile']
    for metric in rrt_metrics:

        for scenario in scenarios:
            best_dfs = {}
            interval_dfs = {}
            reached_goal_dfs = {}
            for planner in [RRT_STAR, TIME_BASED_RRT]:
                best = 0
                best_df = None
                best_simulation = 0
                best_path = ""
                for world in worlds:
                    for mode in modes:
                        for test in tests:
                                for iteration in range(simulation_runs):
                                    file_path = f"{EXPERIMENT_DATA_PATH}{planner}_{world}_{mode}_{test}_{scenario}_{iteration}/{file}"
                                    if os.path.isfile(file_path):
                                        df = pd.read_csv(file_path)

                                        res = get_metric_value(metric, planner, df)

                                        reached_goal_path = f"{EXPERIMENT_DATA_PATH}{planner}_{world}_{mode}_{test}_{scenario}_{iteration}/reached_goal.csv"

                                        if os.path.isfile(reached_goal_path):
                                            df_goal = pd.read_csv(reached_goal_path)
                                            reached_goal = df_goal["GoalReached"][0]
                                        else:
                                            print("Reached Goal does not exists, abort...")
                                            return

                                        if best_df is None and reached_goal:
                                            best_df = df
                                            best_simulation = iteration
                                            best = res
                                            best_path = f"{EXPERIMENT_DATA_PATH}{planner}_{world}_{mode}_{test}_{scenario}_{best_simulation}"

                                        elif compare_metric(metric, res, best) and reached_goal:
                                            best = res
                                            best_df = df
                                            best_simulation = iteration
                                            best_path = f"{EXPERIMENT_DATA_PATH}{planner}_{world}_{mode}_{test}_{scenario}_{best_simulation}"

                if best_df is not None:
                    best_simulation_path = best_path + "/intervals.csv"
                    goal_reached_file_path = best_path + "/reached_goal.csv"

                    key = f"{scenario}_{planner}"
                    best_dfs[key] = best_df
                    interval_dfs[key] = pd.read_csv(best_simulation_path)
                    reached_goal_dfs[key] = pd.read_csv(goal_reached_file_path)

            if len(best_dfs) != 0:
                plot_metric(scenario, best_dfs, interval_dfs, metric, reached_goal_dfs)
        
        if show:
            plt.show()


def visualize_summary_means(lp_improved_logfile, collision_df, improved_lp_summary_df, rrt_logfile, lp_logfile, rrt_summary_df, lp_summary_df):
    """
    Create a table with the means of the means of the relevant metrics
    """
    
    print("Creating Total Table...")
    
    logfile_headers = [
        "Motion Planner",
        "Total collision duration (Mean)",
        "Total collision duration (Std)",
        "Collisions (Mean)",
        "Collisions (Std)",
        "Survival Time (Mean)",
        "Survival Time (Std)",
        "Planning Cycle Time (Mean)",
        "Planning Cycle Time (Std)",
        "Path Length (Mean)",
        "Path Length (Std)",
        "Reached Goals (Sum)",
        "Max Goals"
    ]
    
    total_df = pd.DataFrame(columns=logfile_headers)

    for planner in planners:

        total_collision_duration_mean = '-'
        total_collision_duration_std = '-'
        collision_mean = '-'
        collision_std = '-'
        survival_time_mean = '-'
        survial_time_std = '-'
        planning_cycle_mean = '-'
        planning_cycle_std = '-'
        path_length_mean = '-'
        path_length_std = '-'
        reached_goals_sum = '-'
        possible_goals = '-'

        if planner == LATTICE_PLANNER_IMPROVED:
            # Extract data from collision_df
            total_collision_duration_mean = np.round(collision_df[collision_df['Motion Planner'] == planner]["Total Collision duration (mean)"].mean(), DECIMALS)
            total_collision_duration_std = np.round(collision_df[collision_df['Motion Planner'] == planner]["Total Collision duration (mean)"].std(), DECIMALS)

            collision_mean = np.round(collision_df[collision_df['Motion Planner'] == planner]["Number of Collision (mean)"].mean(), DECIMALS)
            collision_std = np.round(collision_df[collision_df['Motion Planner'] == planner]["Number of Collision (mean)"].std(), DECIMALS)
            
            survival_time_mean = np.round(collision_df[collision_df['Motion Planner'] == planner]["Survival Time Mean (Time Until First collision)"].mean(), DECIMALS)
            survial_time_std = np.round(collision_df[collision_df['Motion Planner'] == planner]["Survival Time Mean (Time Until First collision)"].std(), DECIMALS)

            # Extract data from logfile_df
            planning_cycle_mean = np.round(lp_improved_logfile[lp_improved_logfile['Motion Planner'] == planner]["Planning Cycle Time (mean)"].mean(), DECIMALS)
            planning_cycle_std = np.round(lp_improved_logfile[lp_improved_logfile['Motion Planner'] == planner]["Planning Cycle Time (mean)"].std(), DECIMALS)

            # Extract data from summary_df
            path_length_mean = np.round(improved_lp_summary_df[improved_lp_summary_df['Motion Planner'] == planner]["acc travelled distance (mean)"].mean(), DECIMALS)
            path_length_std = np.round(improved_lp_summary_df[improved_lp_summary_df['Motion Planner'] == planner]["acc travelled distance (mean)"].std(), DECIMALS)

            reached_goals_sum = improved_lp_summary_df[improved_lp_summary_df['Motion Planner'] == planner]["reached goal?"].sum()
            possible_goals = len(improved_lp_summary_df['Motion Planner'] == planner) * simulation_runs

        elif planner == LATTICE_PLANNER:
            # Extract data from collision_df
            total_collision_duration_mean = np.round(collision_df[collision_df['Motion Planner'] == planner]["Total Collision duration (mean)"].mean(), DECIMALS)
            total_collision_duration_std = np.round(collision_df[collision_df['Motion Planner'] == planner]["Total Collision duration (mean)"].std(), DECIMALS)

            collision_mean = np.round(collision_df[collision_df['Motion Planner'] == planner]["Number of Collision (mean)"].mean(), DECIMALS)
            collision_std = np.round(collision_df[collision_df['Motion Planner'] == planner]["Number of Collision (mean)"].std(), DECIMALS)
            
            survival_time_mean = np.round(collision_df[collision_df['Motion Planner'] == planner]["Survival Time Mean (Time Until First collision)"].mean(), DECIMALS)
            survial_time_std = np.round(collision_df[collision_df['Motion Planner'] == planner]["Survival Time Mean (Time Until First collision)"].std(), DECIMALS)

            # Extract data from logfile_df
            planning_cycle_mean = np.round(lp_logfile[lp_logfile['Motion Planner'] == planner]["Planning Cycle Time (mean)"].mean(), DECIMALS)
            planning_cycle_std = np.round(lp_logfile[lp_logfile['Motion Planner'] == planner]["Planning Cycle Time (mean)"].std(), DECIMALS)

            # Extract data from summary_df
            path_length_mean = np.round(lp_summary_df[lp_summary_df['Motion Planner'] == planner]["acc travelled distance (mean)"].mean(), DECIMALS)
            path_length_std = np.round(lp_summary_df[lp_summary_df['Motion Planner'] == planner]["acc travelled distance (mean)"].std(), DECIMALS)

            reached_goals_sum = lp_summary_df[lp_summary_df['Motion Planner'] == planner]["reached goal?"].sum()
            possible_goals = len(lp_summary_df['Motion Planner'] == planner) * simulation_runs
        
        elif planner == RRT_STAR or planner == TIME_BASED_RRT:
            # Extract data from collision_df
            total_collision_duration_mean = np.round(collision_df[collision_df['Motion Planner'] == planner]["Total Collision duration (mean)"].mean(), DECIMALS)
            total_collision_duration_std = np.round(collision_df[collision_df['Motion Planner'] == planner]["Total Collision duration (mean)"].std(), DECIMALS)

            collision_mean = np.round(collision_df[collision_df['Motion Planner'] == planner]["Number of Collision (mean)"].mean(), DECIMALS)
            collision_std = np.round(collision_df[collision_df['Motion Planner'] == planner]["Number of Collision (mean)"].std(), DECIMALS)
            
            survival_time_mean = np.round(collision_df[collision_df['Motion Planner'] == planner]["Survival Time Mean (Time Until First collision)"].mean(), DECIMALS)
            survial_time_std = np.round(collision_df[collision_df['Motion Planner'] == planner]["Survival Time Mean (Time Until First collision)"].std(), DECIMALS)

            # Extract data from logfile_df
            planning_cycle_mean = np.round(rrt_logfile[rrt_logfile['Motion Planner'] == planner]["Search Time (mean)"].mean(), DECIMALS)
            planning_cycle_std = np.round(rrt_logfile[rrt_logfile['Motion Planner'] == planner]["Search Time (mean)"].std(), DECIMALS)

            # Extract data from summary_df
            path_length_mean = np.round(rrt_summary_df[rrt_summary_df['Motion Planner'] == planner]["acc_distance_mean"].mean(), DECIMALS)
            path_length_std = np.round(rrt_summary_df[rrt_summary_df['Motion Planner'] == planner]["acc_distance_mean"].std(), DECIMALS)

            reached_goals_sum = rrt_summary_df[rrt_summary_df['Motion Planner'] == planner]["reached goal?"].sum()
            possible_goals = (len(rrt_summary_df['Motion Planner'] == planner) * simulation_runs) / 2 # Since RRT and TIME_BASED_RRT are in the same table

        else:
            print(f"Unknown planner {planner}")

    
        # Insert data into total_df
        list_row = [planner, total_collision_duration_mean, total_collision_duration_std, collision_mean, collision_std, survival_time_mean, survial_time_std, planning_cycle_mean, planning_cycle_std, path_length_mean, path_length_std, int(reached_goals_sum), int(possible_goals)]
        total_df.loc[len(total_df)] = list_row

    table = tabulate(total_df, headers=logfile_headers, tablefmt="fancy_grid")

    with open(table_path + "total.txt", 'w') as f:
        f.write(table)


if __name__ == '__main__':
    if not os.path.exists(data_path):
        os.makedirs(data_path)

    if not os.path.exists(table_path):
        os.makedirs(table_path)

    if not os.path.exists(plots_path):
        os.makedirs(plots_path)

    # Tables
    rrt_logfile = visualize_logfile_rrt()
    lp_logfile = visualize_logfile_lp()
    lp_improved_logfile = visualize_logfile_lp_improved()
    
    # Collisions
    collision_df = visualize_collision(lp_improved_logfile, rrt_logfile, lp_logfile)

    # I don't think these provides much insight, it just the sum of the values in logfile
    rrt_summary_df = visualize_summary_rrt() 
    lp_summary_df = visualize_summary_lp() 
    improved_lp_summary_df = visualize_summary_lp_improved()

    # Plots
    #plot_metric_average(False)
    #plot_common_metrics(False)
    #plot_rrt_metrics(False)
    #plot_lattice_planner_metrics(False)

    # Table, means of means with relevant metrics, only extracts lattice_planner_improved currently
    visualize_summary_means(lp_improved_logfile, collision_df, improved_lp_summary_df,rrt_logfile, lp_logfile, rrt_summary_df, lp_summary_df)
    



