import subprocess
import time
import yaml
import shutil
import os
import signal
import math
import select
from datetime import datetime
import csv
from colorama import Fore, Style

NEW_TAB_CMD = 'gnome-terminal --tab -- {}'

# Builds a docker image from a image name
def build_image(image_name):
    docker_build_command = ["./dev_env.sh", "build", image_name]
    subprocess.run(docker_build_command, check=True)
    print(f"Docker image {image_name} has been built")

# Open a new terminal tab, run the docker image and start the simulation environment
def simulation(motion_planner, image_name, world, mode, start_pose, test, scenario, scenario_parameter, location_tolerance, dji_safety_radius):
    
    sleep_time = 3

    # catkin build
    #DOCKER_RUN_CMD_CATKIN = "./dev_env.sh start {0} catkin_build.sh".format(image_name)
    #subprocess.run(NEW_TAB_CMD.format(DOCKER_RUN_CMD_CATKIN), shell=True)
    #time.sleep(10)

    if(motion_planner == 'lattice_planner'):
        # lattice_planner
        DOCKER_RUN_CMD_CORE = "./dev_env.sh start {0} /home/{0}/start_scripts/start_lattice_planner.sh {1} {2}".format(image_name, location_tolerance[image_name], dji_safety_radius)
        subprocess.run(NEW_TAB_CMD.format(DOCKER_RUN_CMD_CORE), shell=True)
        time.sleep(sleep_time)
    elif (motion_planner == 'time_based_rrt'):
        # time based rrt_star
        DOCKER_RUN_CMD_RRT = "./dev_env.sh start {0} /home/{0}/start_scripts/start_rrt_star.sh {1} {2}".format(image_name, location_tolerance[image_name], dji_safety_radius)
        subprocess.run(NEW_TAB_CMD.format(DOCKER_RUN_CMD_RRT), shell=True)
        time.sleep(sleep_time)
    elif (motion_planner == 'lattice_planner_improved'):
        # lattice_planner improved
        DOCKER_RUN_CMD_CORE = "./dev_env.sh start {0} /home/{0}/start_scripts/start_lattice_planner.sh {1} {2}".format(image_name, location_tolerance[image_name], dji_safety_radius)
        subprocess.run(NEW_TAB_CMD.format(DOCKER_RUN_CMD_CORE), shell=True)
        time.sleep(sleep_time)
    elif (motion_planner == 'rrt_star'):
        # static rrt_star
        DOCKER_RUN_CMD_RRT = "./dev_env.sh start {0} /home/{0}/start_scripts/start_rrt_star.sh {1} {2}".format(image_name, location_tolerance[image_name], dji_safety_radius)
        subprocess.run(NEW_TAB_CMD.format(DOCKER_RUN_CMD_RRT), shell=True)
        time.sleep(sleep_time)
    elif (motion_planner == 'A-star-low-res'):
        # A-star-low-res
        use_only_geometric_search = True
        use_stand_still = False
        use_predictions = False
        use_geometric_secondary_search = False

        DOCKER_RUN_CMD_CORE = "./dev_env.sh start {0} /home/{0}/start_scripts/start_lattice_planner.sh {1} {2} {3} {4} {5} {6}".format(image_name, location_tolerance[image_name], dji_safety_radius, use_only_geometric_search, use_stand_still, use_predictions, use_geometric_secondary_search)
        subprocess.run(NEW_TAB_CMD.format(DOCKER_RUN_CMD_CORE), shell=True)
        time.sleep(sleep_time)
    elif (motion_planner == 'A-star-high-res'):
        # A-star-high-res
        use_only_geometric_search = False
        use_stand_still = False
        use_predictions = False
        use_geometric_secondary_search = False

        DOCKER_RUN_CMD_CORE = "./dev_env.sh start {0} /home/{0}/start_scripts/start_lattice_planner.sh {1} {2} {3} {4} {5} {6}".format(image_name, location_tolerance[image_name], dji_safety_radius, use_only_geometric_search, use_stand_still, use_predictions, use_geometric_secondary_search)
        subprocess.run(NEW_TAB_CMD.format(DOCKER_RUN_CMD_CORE), shell=True)
        time.sleep(sleep_time)
    else:
        print(Fore.RED + f"Unknown motion planner: {motion_planner}")
        print("Terminating experiment...")
        print(Style.RESET_ALL)
        return False

    # Gazebo
    DOCKER_RUN_CMD_GAZEBO = "./dev_env.sh exec {0} /home/{0}/start_scripts/start_m100_simulator_interface.sh {1} {2}".format(image_name, world, mode)
    subprocess.run(NEW_TAB_CMD.format(DOCKER_RUN_CMD_GAZEBO), shell=True)
    time.sleep(sleep_time)
    
    # Spawn the dji100
    DOCKER_RUN_CMD_DJI100 = "./dev_env.sh exec {0} /home/{0}/start_scripts/spawn_dji100.sh {1} {2} {3}".format(image_name, start_pose[0], start_pose[1], start_pose[2], "dji0")
    subprocess.run(NEW_TAB_CMD.format(DOCKER_RUN_CMD_DJI100), shell=True)
    time.sleep(sleep_time)

    # Motion model simuator
    DOCKER_RUN_CMD_MM_SIM = "./dev_env.sh exec {0} /home/{0}/start_scripts/start_motion_model_simulator.sh {1} {2} {3}".format(image_name, start_pose[0], start_pose[1], start_pose[2])
    subprocess.run(NEW_TAB_CMD.format(DOCKER_RUN_CMD_MM_SIM), shell=True)
    time.sleep(sleep_time)

    # MPC
    DOCKER_RUN_CMD_MPC = "./dev_env.sh exec {0} /home/{0}/start_scripts/start_mpc.sh".format(image_name)
    subprocess.run(NEW_TAB_CMD.format(DOCKER_RUN_CMD_MPC), shell=True)
    time.sleep(sleep_time)

    # Converter mav_dji_sdk
    DOCKER_RUN_CMD_CONV = "./dev_env.sh exec {0} /home/{0}/start_scripts/start_converter_djisdk_mav_node.sh".format(image_name)
    subprocess.run(NEW_TAB_CMD.format(DOCKER_RUN_CMD_CONV), shell=True)
    time.sleep(sleep_time)
  
    # Octomap Ground truth - Currently not in use, ground truth does not exist for all maps
    #DOCKER_RUN_CMD_OCTOMAP_GT = "./dev_env.sh exec {0} /home/{0}/start_scripts/start_octomap_ground_truth.sh".format(image_name)
    #subprocess.run(NEW_TAB_CMD.format(DOCKER_RUN_CMD_OCTOMAP_GT), shell=True)
    #time.sleep(sleep_time)

    # Octomap server
    DOCKER_RUN_OCTOMAP = "./dev_env.sh exec {0} /home/{0}/start_scripts/start_octomap.sh".format(image_name)
    subprocess.run(NEW_TAB_CMD.format(DOCKER_RUN_OCTOMAP), shell=True)
    time.sleep(sleep_time)

    # laser_collector
    DOCKER_RUN_LASER = "./dev_env.sh exec {0} /home/{0}/start_scripts/start_laser_collector.sh".format(image_name)
    subprocess.run(NEW_TAB_CMD.format(DOCKER_RUN_LASER), shell=True)
    time.sleep(sleep_time)

    
    # Converter fron Gazebo to Rviz
    DOCKER_RUN_GAZEBO2RVIZ = "./dev_env.sh exec {0} /home/{0}/start_scripts/start_gazebo2rviz.sh".format(image_name)
    subprocess.run(NEW_TAB_CMD.format(DOCKER_RUN_GAZEBO2RVIZ), shell=True)
    time.sleep(sleep_time)
    
    # Rviz
    DOCKER_RUN_RVIZ = "./dev_env.sh exec {0} /home/{0}/start_scripts/start_rviz.sh".format(image_name)
    subprocess.run(NEW_TAB_CMD.format(DOCKER_RUN_RVIZ), shell=True)
    time.sleep(sleep_time)

    # Scenario Setup
    if(motion_planner == 'lattice_planner'):
        # lattice_planner
        DOCKER_RUN_CMD_SCENARIO = "./dev_env.sh exec {0} /home/{0}/start_scripts/start_scenario.sh {1} {2} {3} {4} {5}".format(image_name, test, scenario, scenario_parameter, location_tolerance['setup_scenario'], dji_safety_radius)
        subprocess.run(NEW_TAB_CMD.format(DOCKER_RUN_CMD_SCENARIO), shell=True)
        time.sleep(sleep_time)
    elif (motion_planner == 'time_based_rrt'):
        # rrt_star
        DOCKER_RUN_CMD_SCENARIO = "./dev_env.sh exec {0} /home/{0}/start_scripts/start_scenario.sh {1} {2} {3} {4} {5}".format(image_name, test, scenario, scenario_parameter, location_tolerance['setup_scenario_time_based_rrt'], dji_safety_radius)
        subprocess.run(NEW_TAB_CMD.format(DOCKER_RUN_CMD_SCENARIO), shell=True)
        time.sleep(sleep_time)
    elif (motion_planner == 'lattice_planner_improved'):
        # lattice_planner improved
        DOCKER_RUN_CMD_SCENARIO = "./dev_env.sh exec {0} /home/{0}/start_scripts/start_scenario.sh {1} {2} {3} {4} {5}".format(image_name, test, scenario, scenario_parameter, location_tolerance['setup_scenario'], dji_safety_radius)
        subprocess.run(NEW_TAB_CMD.format(DOCKER_RUN_CMD_SCENARIO), shell=True)
        time.sleep(sleep_time)
    elif (motion_planner == 'rrt_star'):
        # static rrt_star
        DOCKER_RUN_CMD_SCENARIO = "./dev_env.sh exec {0} /home/{0}/start_scripts/start_scenario.sh {1} {2} {3} {4} {5}".format(image_name, test, scenario, scenario_parameter, location_tolerance['setup_scenario_time_based_rrt'], dji_safety_radius)
        subprocess.run(NEW_TAB_CMD.format(DOCKER_RUN_CMD_SCENARIO), shell=True)
        time.sleep(sleep_time)
    elif (motion_planner == 'A-star-low-res'):
         # A-star-low-res
        DOCKER_RUN_CMD_SCENARIO = "./dev_env.sh exec {0} /home/{0}/start_scripts/start_scenario.sh {1} {2} {3} {4} {5}".format(image_name, test, scenario, scenario_parameter, location_tolerance['setup_scenario'], dji_safety_radius)
        subprocess.run(NEW_TAB_CMD.format(DOCKER_RUN_CMD_SCENARIO), shell=True)
        time.sleep(sleep_time)
    elif (motion_planner == 'A-star-high-res'):
         # A-star-high-res
        DOCKER_RUN_CMD_SCENARIO = "./dev_env.sh exec {0} /home/{0}/start_scripts/start_scenario.sh {1} {2} {3} {4} {5}".format(image_name, test, scenario, scenario_parameter, location_tolerance['setup_scenario'], dji_safety_radius)
        subprocess.run(NEW_TAB_CMD.format(DOCKER_RUN_CMD_SCENARIO), shell=True)
        time.sleep(sleep_time)
    else:
        print(Fore.RED + f"Unknown motion planner: {motion_planner}")
        print("Terminating experiment...")
        print(Style.RESET_ALL)
        return False
    
    # Bash
    DOCKER_RUN_CMD_SCENARIO = "./dev_env.sh bash {0}".format(image_name)
    subprocess.run(NEW_TAB_CMD.format(DOCKER_RUN_CMD_SCENARIO), shell=True)     
    return True

def kill_simulation(image_name):
    print("Killing Simulation")
    DOCKER_EXEC_CMD_SHUTDOWN = "./dev_env.sh exec {0} terminate.sh".format(image_name)
    subprocess.run(NEW_TAB_CMD.format(DOCKER_EXEC_CMD_SHUTDOWN), shell=True)

def send_nav_goal(image_name, end_pose):
    print(f"Sending goal (x,y,z) = ({end_pose[0]}, {end_pose[1]}, {end_pose[2]})")
    DOCKER_EXEC_CMD_SEND_NAV_GOAL = "./dev_env.sh exec {0} /home/{0}/start_scripts/send_nav_goal.sh {1} {2} {3} {4}".format(image_name, end_pose[0], end_pose[1], end_pose[2], "dji0")
    subprocess.run(NEW_TAB_CMD.format(DOCKER_EXEC_CMD_SEND_NAV_GOAL), shell=True)

def check_reached_nav_goal(image_name, timeout):
    command = ['./dev_env.sh', 'topic', image_name, f'/home/{image_name}/start_scripts/check_nav_goal_reached.sh']
    process = subprocess.Popen(command, stdout=subprocess.PIPE, shell=False, preexec_fn=os.setsid)
    previous_minutes_remaining = None
    poll_obj = select.poll()
    poll_obj.register(process.stdout, select.POLLIN)
    while True:
        minutes_remaining = math.floor((timeout - time.time()) / 60)
        if minutes_remaining != previous_minutes_remaining:
            print("Minutes remaining: ", minutes_remaining)
            previous_minutes_remaining = minutes_remaining
        poll_result = poll_obj.poll(0)
        if poll_result:
            output = process.stdout.readline().decode("utf-8").strip()
            if "data:" in output:
                output = output.split("data: ")[1]  # extract the boolean value as a string
                output = True if output == "True" else False  # convert the string to a boolean
                # If planner is done, exit
                if output == True:
                    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                    return True     

        if time.time() > timeout:
            # If the timeout has been reached, terminate the process
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            return False

        # Wait for the subprocess to finish
        process.poll()
        if process.returncode is not None:
            return False

#Read the experiments_mp yaml file to get configuration parameters
with open('sdmp_parameters.yaml', 'r') as file:
    try:
        data = yaml.safe_load(file)
    except yaml.YAMLError as e:
        print(e)

# Extract experiment parameters settings
print("Extracting experiment parameters settings from experiments_mp.yaml")
run_time = data['max_time']
N_simulations = data['simulation_runs']
motion_planners = data['motion_planners']
worlds = data['worlds']
modes = data['modes']
tests = data['tests']

static_scenarios = data['static_scenarios']
dynamic_scenarios = data['dynamic_scenarios']

if static_scenarios is None:
    static_scenarios = []

if dynamic_scenarios is None:
    dynamic_scenarios = []

scenarios = static_scenarios + dynamic_scenarios

scenario_parameter = data['scenario_parameter']
location_tolerance = data['location_tolerance']
dji_safety_radius = data['dji_safety_radius']

print("\n---------------- EXPERIMENT SETUP ----------------")
print(f"Running every experiment for {run_time/60} Minutes ({run_time} Seconds)")
print(f"Running with the following motion planners: {' '.join(motion_planners)}")
print(f"Running with the following worlds: {' '.join(worlds)}")
print(f"Running with the following modes: {' '.join(modes)}")
print(f"Running with the following scenarios: {' '.join(scenarios)}")

totalt_experiment_duration = run_time/60 * N_simulations * len(motion_planners) * len(worlds) * len(modes) * len(scenarios)
print(f"Estimated total experiment duration: {totalt_experiment_duration} Minutes (~ {round(totalt_experiment_duration/60, 2)} Hours)")

print("------------------------------------")

# Run experiments
print("\n---------------- Starting experiments -----------------")

start = time.time()

for motion_planner in motion_planners:
    print(f"--- Starting planning with {motion_planner} ---")

    # Docker image name
    image_name = motion_planners[motion_planner]['image']

    # Build docker image
    build_image(image_name)

    for world in worlds:
        for mode in modes:
            for test in tests:
                for scenario in scenarios:
                    for iter in range(N_simulations):
                        print(f"------- Iteration {iter} for world: {world} in mode: {mode} with test: {test} in scenario: {scenario}  -------")

                        #Extract the spawn position in the world if we need it
                        start_pose = (data['start_position'][world][0], data['start_position'][world][1], data['start_position'][world][2])
                        end_pose = (data['end_position'][world][0], data['end_position'][world][1], data['end_position'][world][2])
                        
                        if(not scenario):
                            print(f"Start position: {start_pose}")
                            print(f"End position: {end_pose}")

                        #Start the simulation environment
                        print("Starting simulation environment...")
                        
                        success = simulation(motion_planner, image_name, world, mode, start_pose, test, scenario, scenario_parameter, location_tolerance, dji_safety_radius)    

                        if not success:
                            print(Fore.RED + f"Failed to build simulation environment for {motion_planner}. Terminating...")
                            print(Style.RESET_ALL)
                            break

                        print(Fore.GREEN + "Succesfully built simulation environment...")
                        print(Style.RESET_ALL)
                        print("Waiting for nav_goal")

                        # If scenario was set it sends an end goal, otherwise we specify it here
                        if(not scenario):
                            send_nav_goal(image_name, end_pose)

                        #Wait for it to finish or end within time-frame
                        print(f"Running experiment for {run_time/60} minutes...")
                        timeout = time.time() + run_time
                        completed = check_reached_nav_goal(image_name, timeout)
                        if completed:
                            print(Fore.GREEN + "Finished early!")
                        else:
                            print(Fore.YELLOW + "Time is up!")

                        print(Style.RESET_ALL)

                        #TODO Save Octomap?

                        if data['debug']:
                            time.sleep(10000)

                        #Kill the simulation environment
                        kill_simulation(image_name)

                        # Save experiment data
                        src_folder = f"./{image_name}/data/"
                    
                        now = datetime.now()
                        date = now.strftime("%d/%m/%Y %H:%M:%S")

                        if scenario in static_scenarios:
                            dst_folder = f"./experiment_data_sdmp/{motion_planner}_{world}_static_{test}_{scenario}_{iter}"
                        else:
                            dst_folder = f"./experiment_data_sdmp/{motion_planner}_{world}_dynamic_{test}_{scenario}_{iter}"

                        print(f"Saving saving data to {dst_folder}")

                        # Create the destination directory if it doesn't exist
                        if not os.path.exists(dst_folder):
                            os.makedirs(dst_folder)

                        # Copy the files from the source directory to the destination directory
                        for file_name in os.listdir(src_folder):
                            src_file = os.path.join(src_folder, file_name)
                            dst_file = os.path.join(dst_folder, file_name)
                            shutil.copy(src_file, dst_file)

                        # Write if we reached goal
                        csv_file_path = os.path.join(dst_folder, "reached_goal.csv")

                        if not os.path.exists(csv_file_path):

                            # Create and write to the CSV file
                            with open(csv_file_path, mode='w', newline='') as file:
                                writer = csv.writer(file)
                                writer.writerow(["GoalReached"])  # Writing the header
                        
                        with open(csv_file_path, mode='a', newline='') as file:
                            writer = csv.writer(file)
                            writer.writerow([int(completed)])  


                        #Wait 10 seconds for Gazebo to close properly
                        print("Wait 10 seconds for Gazebo to close...")
                        time.sleep(10)


        
    print("----------------------------------------")
    print(f"--- Experiment Simulation Completed ---")

end = time.time()
print(f"--- Experiment took {round((end - start)/3600, 2)} hours to run ---")



