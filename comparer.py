BLUE = "\033[94m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
RESET = "\033[0m"
RED = "\033[91m"


def compare_environment_and_launch(env_ir, launch_ir):
    launch_nodes_dict = {}
    launch_nodes = set()
    for action in launch_ir["actions"]:
        if action["type"] == "Node":
            full_name = action.get("name")  # already fully qualified
            exe = action.get("executable", None)
            if full_name:
                launch_nodes.add(full_name)
                launch_nodes_dict[full_name] = exe

    runtime_nodes = set(n["full_name"] for n in env_ir["nodes"])

    print(f"Launch nodes: {launch_nodes}")
    print(f"Runtime nodes: {runtime_nodes}")

    to_be_added = launch_nodes - runtime_nodes
    common_nodes = launch_nodes & runtime_nodes
    not_in_launch = runtime_nodes - launch_nodes

    return {
        "to_be_added": list(to_be_added),
        "common_nodes": list(common_nodes),
        "not_in_launch": list(not_in_launch),
        "launch_nodes_dict": launch_nodes_dict,
    }


def print_comparison_result(result):
    """"""
    # TODO: send the results to ROS2LaunchParent class
    launch_nodes_dict = result.get("launch_nodes_dict", {})

    print(f" {RESET} Comparison Result:")
    if result["to_be_added"]:  # start
        print(f"{GREEN}[DEBUG]{RESET} New nodes will be added:")
        for full_name in result["to_be_added"]:
            exe = launch_nodes_dict.get(full_name)
            if exe:
                print(f"{GREEN}    + (executable={exe})")
            else:
                print(f"{GREEN}    + {full_name}")
    else:
        print(f" {YELLOW}  No new nodes will be added.")

    if result["common_nodes"]:  # no action
        print(f" {RESET} Nodes already running (common):")
        for full_name in result["common_nodes"]:
            exe = launch_nodes_dict.get(full_name)
            if exe:
                print(f"    - {full_name} (executable={exe})")
            else:
                print(f"    - {full_name}")
    else:
        print("  No nodes are common.")

    if result["not_in_launch"]:  # kill
        print(f"  {RESET} Nodes running but not in incoming launch description:")
        for full_name in result["not_in_launch"]:
            print(f"{RED}    - {full_name} {RESET}")
    else:
        print("  All runtime nodes are included in the incoming launch.")
