"""This file configs the simulation and starts it"""

# ==================== imports ====================
import os
import json
import subprocess

import sim_utils


# ==================== the main simulation func ====================
def main():
    args = sim_utils.parse_arguments()

    cur_launch_config = sim_utils.DEFAULT_LAUNCH_CONFIG.copy()
    cur_launch_config["headless"] = args.headless
    os.environ["LAUNCH_CONFIG"] = json.dumps(cur_launch_config)

    from sim_app import Simulation          # import sim app after setting the launch config

    usds_to_add = sim_utils.get_usds_to_add(args, sim_utils.PROJECT_HOME_DIR)

    simulation = Simulation(args.usd_path, usds_to_add)

    libs_to_run_json = json.dumps(sim_utils.get_libs_to_add(args))
    print(f"[main_sim] Starting LibManager with libs: {libs_to_run_json}")
    lib_proc = subprocess.Popen(["/usr/bin/python3",    # running on system python instead of isaac python to use unsupported packages
                                 os.path.join(os.path.dirname(os.path.abspath(__file__)), "lib_manager.py"),
                                 "--libs",
                                 libs_to_run_json])

    simulation.run_simulation()

    lib_proc.terminate()


# ==================== run the main ====================
if __name__ == "__main__":
    main()
