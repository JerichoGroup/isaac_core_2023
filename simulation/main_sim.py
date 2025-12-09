"""This file configs the simulation and starts it"""

# ==================== imports ====================
import os
import json
import sim_utils


# ==================== the main simulation func ====================
def main():
    """"
    Main function to configure and start the simulation.
    """

    args = sim_utils.parse_arguments()

    sim_utils.start_header_injector_if_enabled(args)

    cur_launch_config = sim_utils.DEFAULT_LAUNCH_CONFIG.copy()
    cur_launch_config["headless"] = args.headless
    os.environ["LAUNCH_CONFIG"] = json.dumps(cur_launch_config)

    from sim_app import Simulation   # import sim app after setting the launch config

    usds_to_add = sim_utils.get_usds_to_add(args, sim_utils.PROJECT_HOME_DIR)

    simulation = Simulation(args.usd_path, usds_to_add)

    simulation.run_simulation()


# ==================== run the main ====================
if __name__ == "__main__":
    main()
