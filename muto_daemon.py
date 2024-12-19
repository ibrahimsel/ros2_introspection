#!/usr/bin/env python3

import sys
import os
import json
import psutil
from dataclasses import dataclass, field, asdict
from typing import Dict, Any, List, Optional
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from ament_index_python.packages import (
    get_package_prefix,
    PackageNotFoundError,
)

from rcl_interfaces.srv import ListParameters, GetParameters

# ANSI color codes for pretty printing
BLUE = "\033[94m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
RESET = "\033[0m"
RED = "\033[91m"


@dataclass(frozen=True)
class NodeInfo:
    name: str
    namespace: str
    package: Optional[str] = None
    executable: Optional[str] = None
    launch_arguments: Dict[str, Any] = field(default_factory=dict)
    parameters: Dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class ComposableNodeInfo:
    name: str
    namespace: str
    package: Optional[str] = None
    executable: Optional[str] = None
    launch_arguments: Dict[str, Any] = field(default_factory=dict)
    parameters: Dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class ContainerInfo:
    name: str
    namespace: str
    package: Optional[str] = None
    executable: Optional[str] = None
    launch_arguments: Dict[str, Any] = field(default_factory=dict)
    parameters: Dict[str, Any] = field(default_factory=dict)


def get_package_from_executable(executable_path: str) -> Optional[str]:
    """
    Attempts to infer the ROS 2 package name from the executable's path.
    Assumes that the package name is the directory following '/lib/' in the path.
    """
    try:
        # Normalize the path
        normalized_path = os.path.normpath(executable_path)
        parts = normalized_path.split(os.sep)
        if "lib" in parts:
            lib_index = parts.index("lib")
            if lib_index + 1 < len(parts):
                package_name = parts[lib_index + 1]
                return package_name
        return None
    except Exception as e:
        print(
            f"{RED}Error inferring package from executable path '{executable_path}': {e}{RESET}"
        )
        return None


def get_process_info(node_full_name: str) -> Optional[Dict[str, str]]:
    """
    Attempts to map a ROS 2 node full name to a system process to identify the executable and executable path.
    Returns a dictionary with 'name' and 'exe_path'.
    """
    for proc in psutil.process_iter(["pid", "name", "cmdline", "exe"]):
        try:
            # Check if the node full name appears in the command line arguments
            if any(node_full_name in cmd for cmd in proc.info["cmdline"]):
                return {"name": proc.info["name"], "exe_path": proc.info["exe"]}
            # Alternatively, match the executable name directly
            node_name = node_full_name.split("/")[-1]
            if node_name in proc.info["name"]:
                return {"name": proc.info["name"], "exe_path": proc.info["exe"]}
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            continue
    return None


def extract_parameters(
    node: Node, target_node_name: str, target_node_namespace: str
) -> Dict[str, Any]:
    """
    Extracts parameters for a given ROS 2 node using a Parameter Client.
    """
    parameters = {}
    try:
        # Ensure namespace ends with '/'
        if not target_node_namespace.endswith("/"):
            target_node_namespace += "/"

        # Construct the service names correctly
        list_params_service = os.path.join(
            target_node_namespace, target_node_name, "list_parameters"
        )
        get_params_service = os.path.join(
            target_node_namespace, target_node_name, "get_parameters"
        )

        # Create a Parameter Client for the target node's ListParameters service
        parameter_client = node.create_client(ListParameters, list_params_service)

        # Wait for the service to be available
        if not parameter_client.wait_for_service(timeout_sec=2.0):
            print(
                f"{RED}ListParameters service not available for node '{target_node_name}' in namespace '{target_node_namespace}'.{RESET}"
            )
            return parameters

        # Prepare the request to list all parameters
        request = ListParameters.Request()
        request.prefixes = [""]
        request.depth = 10  # Adjust as needed

        # Send the request
        future = parameter_client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

        if not future.done():
            print(
                f"{RED}Failed to retrieve parameters for node '{target_node_name}' in namespace '{target_node_namespace}'.{RESET}"
            )
            return parameters

        if future.result() is None:
            print(
                f"{RED}Service call failed for ListParameters of node '{target_node_name}' in namespace '{target_node_namespace}'.{RESET}"
            )
            return parameters

        param_names = future.result().names

        if not param_names:
            # No parameters found
            return parameters

        # Create a GetParameters client to retrieve parameter values
        get_params_client = node.create_client(GetParameters, get_params_service)

        if not get_params_client.wait_for_service(timeout_sec=2.0):
            print(
                f"{RED}GetParameters service not available for node '{target_node_name}' in namespace '{target_node_namespace}'.{RESET}"
            )
            return parameters

        get_request = GetParameters.Request()
        get_request.names = param_names

        get_future = get_params_client.call_async(get_request)
        rclpy.spin_until_future_complete(node, get_future, timeout_sec=5.0)

        if not get_future.done():
            print(
                f"{RED}Failed to get parameters for node '{target_node_name}' in namespace '{target_node_namespace}'.{RESET}"
            )
            return parameters

        get_response = get_future.result()

        if get_response is None:
            print(
                f"{RED}GetParameters service call returned None for node '{target_node_name}' in namespace '{target_node_namespace}'.{RESET}"
            )
            return parameters

        for param, value in zip(get_response.names, get_response.values):
            parameters[param] = value

    except Exception as e:
        print(
            f"{RED}Exception while retrieving parameters for node '{target_node_name}' in namespace '{target_node_namespace}': {e}{RESET}"
        )

    return parameters


def build_executable_package_map() -> Dict[str, str]:
    """
    Builds a mapping from executable names to package names using ament_index_python.
    """
    import ament_index_python.packages

    executable_package_map = {}
    try:
        packages = ament_index_python.packages.get_packages_with_prefixes()
        for package in packages.keys():
            try:
                share_dir = ament_index_python.packages.get_package_share_directory(
                    package
                )
                prefix = get_package_prefix(package)
                lib_dir = os.path.join(prefix, "lib", package)
                if os.path.isdir(lib_dir):
                    for exe in os.listdir(lib_dir):
                        executable_path = os.path.join(lib_dir, exe)
                        if os.path.isfile(executable_path) and os.access(
                            executable_path, os.X_OK
                        ):
                            executable_package_map[exe] = package
            except PackageNotFoundError:
                # Package not found, skip
                continue
            except Exception:
                # Skip packages that do not follow the standard structure
                continue
    except Exception as e:
        print(f"{RED}Error building executable-package map: {e}{RESET}")
    return executable_package_map


def introspect_ros2_runtime() -> Dict[str, Any]:
    """
    Introspects the current ROS 2 runtime and collects information about nodes, composable nodes, and containers.
    """
    rclpy.init()
    try:
        # Build executable-package mapping
        executable_package_map = build_executable_package_map()

        # Create a temporary node to interact with the ROS 2 system
        temp_node = Node("_runtime_introspection_node")
        executor = SingleThreadedExecutor()
        executor.add_node(temp_node)

        # Allow some time for the node to initialize and discover the graph
        rclpy.spin_once(temp_node, timeout_sec=1)

        # Get list of all nodes with their namespaces
        node_info_list = temp_node.get_node_names_and_namespaces()
        print(f"{GREEN}Discovered {len(node_info_list)} active ROS 2 nodes.{RESET}")

        nodes_info: List[NodeInfo] = []
        composable_nodes_info: List[ComposableNodeInfo] = []
        containers_info: List[ContainerInfo] = []

        for name, namespace in node_info_list:
            try:
                # Skip internal or introspection nodes (names starting with '_')
                if name.startswith("_"):
                    continue

                # Construct the full node name
                if namespace == "/":
                    node_full_name = f"/{name}"
                else:
                    node_full_name = f"{namespace}/{name}"

                # Extract parameters using Parameter Client
                parameters = extract_parameters(temp_node, name, namespace)

                # Attempt to identify if the node is part of a ComposableNodeContainer
                # This is a heuristic approach; adjust as needed
                is_composable = False
                is_container = False

                # Check if the node name indicates it's a container
                # This is a heuristic approach. # TODO: Replace with ros2component.api methods
                if "component_container" in name.lower() or "container" in name.lower():
                    is_container = True

                # Get process info
                process_info = get_process_info(node_full_name)
                executable = process_info["name"] if process_info else None
                exe_path = process_info["exe_path"] if process_info else None

                # Infer package name from executable path
                package = get_package_from_executable(exe_path) if exe_path else None

                # If package is still None, try mapping via executable name
                if not package and executable in executable_package_map:
                    package = executable_package_map[executable]

                if is_container:
                    # It's a ComposableNodeContainer
                    container_info = ContainerInfo(
                        name=name,
                        namespace=namespace,
                        package=package,
                        executable=executable,
                        launch_arguments={},  # Launch arguments are not directly accessible
                        parameters=parameters,
                    )
                    containers_info.append(container_info)
                else:
                    # It's a regular Node or a ComposableNode
                    # Heuristic: if the node shares a process with a container, it's a ComposableNode
                    is_composable = False
                    for container in containers_info:
                        if container.executable and container.executable == executable:
                            is_composable = True
                            break

                    if is_composable:
                        composable_node_info = ComposableNodeInfo(
                            name=name,
                            namespace=namespace,
                            package=package,
                            executable=executable,
                            launch_arguments={},  # Launch arguments are not directly accessible
                            parameters=parameters,
                        )
                        composable_nodes_info.append(composable_node_info)
                    else:
                        node_info = NodeInfo(
                            name=name,
                            namespace=namespace,
                            package=package,
                            executable=executable,
                            launch_arguments={},  # Launch arguments are not directly accessible
                            parameters=parameters,
                        )
                        nodes_info.append(node_info)

            except Exception as e:
                print(
                    f"{RED}Error processing node '{name}' in namespace '{namespace}': {e}{RESET}"
                )
                continue

        # Clean up
        temp_node.destroy_node()
        rclpy.shutdown()

        # Compile the collected information
        runtime_info = {
            "Nodes": [asdict(node) for node in nodes_info],
            "ComposableNodes": [asdict(cn) for cn in composable_nodes_info],
            "Containers": [asdict(c) for c in containers_info],
        }

        return runtime_info
    except Exception as e:
        print(f"Exception: {e}")


def main():
    runtime_info = introspect_ros2_runtime()

    # Pretty print the results
    print(YELLOW + "\nRuntime Introspection Summary:" + RESET)

    print("\nNodes:")
    for node in runtime_info["Nodes"]:
        print(json.dumps(node, indent=4))

    print("\nComposableNodes:")
    for cn in runtime_info["ComposableNodes"]:
        print(json.dumps(cn, indent=4))

    print("\nContainers:")
    for container in runtime_info["Containers"]:
        print(json.dumps(container, indent=4))

    # Optionally, save the information to a JSON file
    output_file = "ros2_runtime_introspection.json"
    try:
        with open(output_file, "w") as f:
            json.dump(runtime_info, f, indent=4)
        print(f"\n{GREEN}Runtime introspection data saved to {output_file}{RESET}")
    except Exception as e:
        print(f"{RED}Failed to save introspection data to {output_file}: {e}{RESET}")


if __name__ == "__main__":
    main()
