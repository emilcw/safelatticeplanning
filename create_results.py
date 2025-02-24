import subprocess
from colorama import Fore, Style

"""
Create the visualization results automatically by running this script, or 
open the container with dev_env.sh and start visualize_sdmp.py from the commande line.
"""

NEW_TAB_CMD = 'gnome-terminal --tab -- {}'

# Builds a docker image from a image name
def build_image(image_name):
    docker_build_command = ["./dev_env.sh", "build", image_name]
    subprocess.run(docker_build_command, check=True)
    print(f"Docker image {image_name} has been built")

# Create the results by calling the script in docker container
def create_results(image_name):    
    DOCKER_RUN_VISUALIZE = "./dev_env.sh start {0} /home/{0}/run.sh".format(image_name)
    subprocess.run(NEW_TAB_CMD.format(DOCKER_RUN_VISUALIZE), shell=True)
    return True

print("Creating Tables and Plots from experiments...")

image_name = "visualization"

print("Building image...")
build_image(image_name)

print("Creating results...")
create_results(image_name)

print(Fore.GREEN + "Successfully created plots and tables!")

