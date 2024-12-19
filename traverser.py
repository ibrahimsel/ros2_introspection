import time
import sys
import os
from dataclasses import dataclass, field, asdict
from typing import Dict, Any, List
from launch import LaunchContext
from launch.substitutions import TextSubstitution
from launch.actions import (
    IncludeLaunchDescription,
    GroupAction,
    DeclareLaunchArgument,
)
from launch_ros.actions import PushRosNamespace, Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.utilities import perform_substitutions
from launch.substitution import Substitution
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.substitutions.find_package import FindPackageShare
from launch.substitutions.substitution_failure import SubstitutionFailure

import yaml

BLUE = "\033[94m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
RESET = "\033[0m"
RED = "\033[91m"


@dataclass(frozen=True)
class NodeInfo:
    package: str
    executable: str
    name: str
    namespace: str
    launch_arguments: Dict[str, Any] = field(default_factory=dict)
    param: Dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class ComposableNodeInfo:
    package: str
    plugin: str
    name: str
    launch_arguments: Dict[str, Any] = field(default_factory=dict)
    param: Dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class ContainerInfo:
    package: str
    executable: str
    name: str
    namespace: str
    launch_arguments: Dict[str, Any] = field(default_factory=dict)
    param: Dict[str, Any] = field(default_factory=dict)


def resolve_substitutions(context: LaunchContext, value: Any) -> Any:
    """
    Recursively resolve substitutions in the given value using the provided LaunchContext.
    Handles nested structures like lists and dictionaries.
    """
    if isinstance(value, (list, tuple)):
        # Resolve each item and join them into a comma-separated string
        resolved_list = [resolve_substitutions(context, item) for item in value]
        return "".join(map(str, resolved_list))
    elif isinstance(value, dict):
        # Ensure all keys are strings to prevent unhashable types
        return {
            str(resolve_substitutions(context, k)): resolve_substitutions(context, v)
            for k, v in value.items()
        }
    elif isinstance(value, Substitution):
        try:
            # Perform substitutions and join the results
            resolved = perform_substitutions(context, [value])
            return "".join(resolved)
        except SubstitutionFailure:
            # Return a placeholder and log a warning if substitution fails
            print(f"{RED}Warning: Could not resolve Substitution '{value}'.{RESET}")
            return f"<unresolved_substitution_{value}>"
        except Exception as e:
            # Log any other exceptions during substitution resolution
            print(f"{RED}Error resolving Substitution '{value}': {e}{RESET}")
            return f"<error_resolving_{value}>"
    elif isinstance(value, LaunchConfiguration):
        try:
            # Perform substitutions for LaunchConfiguration
            resolved = perform_substitutions(context, [value])
            return "".join(resolved)
        except SubstitutionFailure:
            # Return a placeholder and log a warning if substitution fails
            print(
                f"{RED}Warning: Could not resolve LaunchConfiguration '{value}'.{RESET}"
            )
            return f"<unresolved_launch_configuration_{value}>"
        except Exception as e:
            # Log any other exceptions during substitution resolution
            print(f"{RED}Error resolving LaunchConfiguration '{value}': {e}{RESET}")
            return f"<error_resolving_{value}>"
    elif isinstance(value, TextSubstitution):
        # Return the text value of the TextSubstitution
        return value.text
    elif isinstance(value, FindPackageShare):
        return value.perform(context)
    else:
        # For all other types, return their string representation
        return str(value)


def extract_parameters_from_file(
    param_file_path: str, context: LaunchContext
) -> Dict[str, Any]:
    """
    Parse a YAML parameter file and extract parameters, resolving any substitutions.
    Assumes ROS 2 parameter file structure with 'ros__parameters' key.
    """
    param_args = {}
    if os.path.isfile(param_file_path):
        try:
            with open(param_file_path, "r") as f:
                params_yaml = yaml.safe_load(f)
                if params_yaml is None:
                    print(
                        f"{YELLOW}Warning: Parameter file {param_file_path} is empty.{RESET}"
                    )
                    return param_args
                for node_key, node_params in params_yaml.items():
                    ros_params = node_params.get("ros__parameters", {})
                    for param_name, param_value in ros_params.items():
                        resolved_param_name = resolve_substitutions(context, param_name)
                        resolved_param_value = resolve_substitutions(
                            context, param_value
                        )
                        param_args[resolved_param_name] = resolved_param_value
        except Exception as e:
            print(f"{RED}Failed to parse parameter file {param_file_path}: {e}{RESET}")
    else:
        print(f"{RED}Parameter file {param_file_path} does not exist.{RESET}")
    return param_args


def extract_node_launch_arguments(node: Node, context: LaunchContext) -> Dict[str, Any]:
    """
    Extract launch arguments relevant to the node by inspecting its parameters and remappings.
    Only captures arguments that are LaunchConfigurations or are directly specified.
    """
    launch_args = {}

    # Access private __parameters attribute
    parameters = getattr(node, "_Node__parameters", [])

    for param in parameters:
        if isinstance(param, dict):
            for key, value in param.items():
                resolved_key = resolve_substitutions(context, key)
                resolved_value = resolve_substitutions(context, value)
                launch_args[resolved_key] = resolved_value
        elif isinstance(param, str):
            # If parameters are provided as file paths, parse the YAML file
            param_file_path = resolve_substitutions(context, param)
            params_from_file = extract_parameters_from_file(param_file_path, context)
            launch_args.update(params_from_file)

    # Access private __remappings attribute
    remappings = getattr(node, "_Node__remappings", [])

    remap_list = []
    for remap in remappings:
        src, dst = remap
        resolved_src = resolve_substitutions(context, src)
        resolved_dst = resolve_substitutions(context, dst)
        remap_entry = f"{resolved_src}:={resolved_dst}"
        remap_list.append(remap_entry)

    if remap_list:
        launch_args["remap"] = "\n".join(remap_list)

    return launch_args


def extract_composable_node_launch_arguments(
    cn: ComposableNode, context: LaunchContext
) -> Dict[str, Any]:
    """
    Extract launch arguments relevant to the composable node by inspecting its parameters.
    Only captures arguments that are LaunchConfigurations or are directly specified.
    """
    launch_args = {}

    # Access private __parameters attribute
    parameters = getattr(cn, "_ComposableNode__parameters", [])

    for param in parameters:
        if isinstance(param, dict):
            for key, value in param.items():
                resolved_key = resolve_substitutions(context, key)
                resolved_value = resolve_substitutions(context, value)
                launch_args[resolved_key] = resolved_value
        elif isinstance(param, str):
            # If parameters are provided as file paths, parse the YAML file
            param_file_path = resolve_substitutions(context, param)
            params_from_file = extract_parameters_from_file(param_file_path, context)
            launch_args.update(params_from_file)

    return launch_args


def extract_container_launch_arguments(
    container: ComposableNodeContainer, context: LaunchContext
) -> Dict[str, Any]:
    """
    Extract launch arguments relevant to the container by inspecting its parameters and remappings.
    Only captures arguments that are LaunchConfigurations or are directly specified.
    """
    launch_args = {}

    # Access private __parameters attribute
    parameters = getattr(container, "_Node__parameters", [])

    for param in parameters:
        if isinstance(param, dict):
            for key, value in param.items():
                resolved_key = resolve_substitutions(context, key)
                resolved_value = resolve_substitutions(context, value)
                launch_args[resolved_key] = resolved_value
        elif isinstance(param, str):
            # If parameters are provided as file paths, parse the YAML file
            param_file_path = resolve_substitutions(context, param)
            params_from_file = extract_parameters_from_file(param_file_path, context)
            launch_args.update(params_from_file)

    # Access private __remappings attribute
    remappings = getattr(container, "_Node__remappings", [])

    remap_list = []
    for remap in remappings:
        src, dst = remap
        resolved_src = resolve_substitutions(context, src)
        resolved_dst = resolve_substitutions(context, dst)
        remap_entry = f"{resolved_src}:={resolved_dst}"
        remap_list.append(remap_entry)

    if remap_list:
        # Join multiple remaps with semicolons
        launch_args["remap"] = ";".join(remap_list)

    return launch_args


def extract_node_parameters(node: Node, context: LaunchContext) -> Dict[str, Any]:
    """
    Extract parameters for a Node entity.
    """
    param_args = {}
    parameters = getattr(node, "_Node__parameters", [])
    for param in parameters:
        if isinstance(param, dict):
            for key, value in param.items():
                resolved_key = resolve_substitutions(context, key)
                resolved_value = resolve_substitutions(context, value)
                param_args[resolved_key] = resolved_value
        elif isinstance(param, str):
            param_file_path = resolve_substitutions(context, param)
            params_from_file = extract_parameters_from_file(param_file_path, context)
            param_args.update(params_from_file)
    return param_args


def extract_composable_node_parameters(
    cn: ComposableNode, context: LaunchContext
) -> Dict[str, Any]:
    """
    Extract parameters for a ComposableNode entity.
    """
    param_args = {}
    parameters = getattr(cn, "_ComposableNode__parameters", [])
    for param in parameters:
        if isinstance(param, dict):
            for key, value in param.items():
                resolved_key = resolve_substitutions(context, key)
                resolved_value = resolve_substitutions(context, value)
                param_args[resolved_key] = resolved_value
        elif isinstance(param, str):
            param_file_path = resolve_substitutions(context, param)
            params_from_file = extract_parameters_from_file(param_file_path, context)
            param_args.update(params_from_file)
    return param_args


def extract_container_parameters(
    container: ComposableNodeContainer, context: LaunchContext
) -> Dict[str, Any]:
    """
    Extract parameters for a ComposableNodeContainer entity.
    """
    param_args = {}
    parameters = getattr(container, "_Node__parameters", [])
    for param in parameters:
        if isinstance(param, dict):
            for key, value in param.items():
                resolved_key = resolve_substitutions(context, key)
                resolved_value = resolve_substitutions(context, value)
                param_args[resolved_key] = resolved_value
        elif isinstance(param, str):
            param_file_path = resolve_substitutions(context, param)
            params_from_file = extract_parameters_from_file(param_file_path, context)
            param_args.update(params_from_file)
    return param_args


def process_node(
    entity: Node, context: LaunchContext, nodes: List[NodeInfo], current_namespace: str
):
    """
    Process a Node entity and append its information to the nodes list.
    """
    try:
        # Resolve package, executable, name, and namespace
        package = resolve_substitutions(
            context, getattr(entity, "_Node__package", "") or ""
        )
        executable = resolve_substitutions(
            context, getattr(entity, "_Node__node_executable", "") or ""
        )
        node_name = resolve_substitutions(
            context, getattr(entity, "_Node__node_name", "") or ""
        )
        node_namespace = resolve_substitutions(
            context, getattr(entity, "_Node__node_namespace", "") or ""
        )

        # If namespace is not set, use current_namespace
        if not node_namespace:
            node_namespace = current_namespace

        # Clean up the full name
        full_name = (
            os.path.join(node_namespace, node_name).replace("\\", "/").rstrip("/")
        )
        if full_name.startswith("/"):
            full_name = full_name[1:]

        print(
            f"Found Node: {GREEN}{executable}{RESET} "
            f"with the full name: {BLUE}{full_name}{RESET} "
            f"within package {YELLOW}{package}{RESET}"
        )

        # Extract launch arguments and parameters
        launch_args = extract_node_launch_arguments(entity, context)
        param_args = extract_node_parameters(entity, context)

        node_info = NodeInfo(
            package=package,
            executable=executable,
            name=node_name,
            namespace=node_namespace,
            launch_arguments=launch_args,
            param=param_args,
        )
        nodes.append(node_info)

    except Exception as e:
        print(f"{RED}Error processing node '{entity}': {e}{RESET}")


def process_composable_node(
    entity: ComposableNode,
    context: LaunchContext,
    composable_nodes: List[ComposableNodeInfo],
):
    """
    Process a ComposableNode entity and append its information to the composable_nodes list.
    """
    try:
        package = resolve_substitutions(
            context, getattr(entity, "_ComposableNode__package", "") or ""
        )
        plugin = resolve_substitutions(
            context, getattr(entity, "_ComposableNode__plugin", "") or ""
        )
        name = resolve_substitutions(
            context, getattr(entity, "_ComposableNode__name", "") or ""
        )

        print(
            f"Found ComposableNode: {GREEN}{plugin}{RESET} name: {BLUE}{name}{RESET} in package {YELLOW}{package}{RESET}"
        )

        # Extract launch arguments and parameters
        launch_args = extract_composable_node_launch_arguments(entity, context)
        param_args = extract_composable_node_parameters(entity, context)

        cn_info = ComposableNodeInfo(
            package=package,
            plugin=plugin,
            name=name,
            launch_arguments=launch_args,
            param=param_args,
        )
        composable_nodes.append(cn_info)

    except Exception as e:
        print(f"{RED}Error processing composable node '{entity}': {e}{RESET}")


def process_container(
    entity: ComposableNodeContainer,
    context: LaunchContext,
    containers: List[ContainerInfo],
    current_namespace: str,
):
    """
    Process a ComposableNodeContainer entity and append its information to the containers list.
    """
    try:
        package = resolve_substitutions(
            context, getattr(entity, "_Node__package", "") or ""
        )
        executable = resolve_substitutions(
            context, getattr(entity, "_Node__node_executable", "") or ""
        )
        container_name = resolve_substitutions(
            context, getattr(entity, "_Node__node_name", "") or ""
        )
        container_namespace = resolve_substitutions(
            context, getattr(entity, "_Node__node_namespace", "") or ""
        )

        # If namespace is not set, use current_namespace
        if not container_namespace:
            container_namespace = current_namespace

        # Clean up the full name
        full_name = (
            os.path.join(container_namespace, container_name)
            .replace("\\", "/")
            .rstrip("/")
        )
        if full_name.startswith("/"):
            full_name = full_name[1:]

        print(
            f"Found ComposableNodeContainer: {GREEN}{executable}{RESET} "
            f"with the full name: {BLUE}{full_name}{RESET} "
            f"within package {YELLOW}{package}{RESET}"
        )

        # Extract launch arguments and parameters
        launch_args = extract_container_launch_arguments(entity, context)
        param_args = extract_container_parameters(entity, context)

        container_info = ContainerInfo(
            package=package,
            executable=executable,
            name=container_name,
            namespace=container_namespace,
            launch_arguments=launch_args,
            param=param_args,
        )
        containers.append(container_info)

    except Exception as e:
        print(f"{RED}Error processing container '{entity}': {e}{RESET}")


def recursively_extract_entities(
    entities: List[Any],
    context: LaunchContext,
    nodes: List[NodeInfo],
    composable_nodes: List[ComposableNodeInfo],
    containers: List[ContainerInfo],
    current_namespace: str = "",
):
    for entity in entities:
        try:
            if isinstance(entity, DeclareLaunchArgument):
                # Declare launch arguments and populate the context
                try:
                    entity.execute(context)
                    arg_name = entity.name
                    arg_default = resolve_substitutions(context, entity.default_value)
                    if arg_default == "":
                        print(
                            f"{GREEN}Declared Launch Argument: {arg_name} = <no default>{RESET}"
                        )
                    else:
                        print(
                            f"{GREEN}Declared Launch Argument: {arg_name} = {arg_default}{RESET}"
                        )
                except Exception as e:
                    print(
                        f"{RED}Error declaring launch argument '{entity.name}': {e}{RESET}"
                    )
                continue  # Move to the next entity

            if isinstance(entity, PushRosNamespace):
                # Resolve the new namespace and update current_namespace
                new_ns = resolve_substitutions(context, entity.namespace)
                updated_namespace = (
                    os.path.join(current_namespace, new_ns)
                    .replace("\\", "/")
                    .rstrip("/")
                )
                print(f"{GREEN}Pushed ROS Namespace: {updated_namespace}{RESET}")
                # Recurse into the namespace
                sub_entities = entity.get_sub_entities()
                recursively_extract_entities(
                    sub_entities,
                    context,
                    nodes,
                    composable_nodes,
                    containers,
                    updated_namespace,
                )
                continue  # Move to the next entity

            if isinstance(entity, IncludeLaunchDescription):
                # Include and process included launch description
                try:
                    included_ld = (
                        entity.launch_description_source.get_launch_description(
                            context=context
                        )
                    )
                    included_ld.visit(context)  # Expand includes, substitutions, etc.
                    recursively_extract_entities(
                        included_ld.entities,
                        context,
                        nodes,
                        composable_nodes,
                        containers,
                        current_namespace,
                    )
                except Exception as e:
                    print(f"{RED}Error including launch description: {e}{RESET}")
                continue  # Move to the next entity

            if isinstance(entity, GroupAction):
                # Process group actions by recursing into their sub_entities
                sub_entities = entity.get_sub_entities()
                recursively_extract_entities(
                    sub_entities,
                    context,
                    nodes,
                    composable_nodes,
                    containers,
                    current_namespace,
                )
                continue  # Move to the next entity

            # Process Node, ComposableNode, and ComposableNodeContainer
            if not already_found(
                entity, context, nodes, composable_nodes, containers, current_namespace
            ):
                if isinstance(entity, Node):
                    process_node(entity, context, nodes, current_namespace)
                elif isinstance(entity, ComposableNode):
                    process_composable_node(entity, context, composable_nodes)
                elif isinstance(entity, ComposableNodeContainer):
                    process_container(entity, context, containers, current_namespace)

        except LookupError as e:
            if isinstance(entity, Node):
                resolved_package = resolve_substitutions(
                    context, getattr(entity, "_Node__package", "") or ""
                )
                print(f"{RED}Package could not be found: {resolved_package}{RESET}")
        except Exception as e:
            print(f"{RED}Error while extracting entities: {e}{RESET}")
            continue


def already_found(
    entity,
    context: LaunchContext,
    nodes: List[NodeInfo],
    composable_nodes: List[ComposableNodeInfo],
    containers: List[ContainerInfo],
    current_namespace: str,
) -> bool:
    """
    Check if the entity has already been processed.
    Compare based on unique attributes to avoid duplicates.
    """
    if isinstance(entity, Node):
        executable = resolve_substitutions(
            context, getattr(entity, "_Node__node_executable", "") or ""
        )
        name = resolve_substitutions(
            context, getattr(entity, "_Node__node_name", "") or ""
        )
        namespace = resolve_substitutions(
            context, getattr(entity, "_Node__node_namespace", "") or ""
        )
        package = resolve_substitutions(
            context, getattr(entity, "_Node__package", "") or ""
        )
        namespace = namespace if namespace else current_namespace
        for n in nodes:
            if (
                n.executable == executable
                and n.name == name
                and n.namespace == namespace
                and n.package == package
            ):
                return True
    elif isinstance(entity, ComposableNode):
        plugin = resolve_substitutions(
            context, getattr(entity, "_ComposableNode__plugin", "") or ""
        )
        name = resolve_substitutions(
            context, getattr(entity, "_ComposableNode__name", "") or ""
        )
        package = resolve_substitutions(
            context, getattr(entity, "_ComposableNode__package", "") or ""
        )
        for cn in composable_nodes:
            if cn.plugin == plugin and cn.name == name and cn.package == package:
                return True
    elif isinstance(entity, ComposableNodeContainer):
        executable = resolve_substitutions(
            context, getattr(entity, "_Node__node_executable", "") or ""
        )
        name = resolve_substitutions(
            context, getattr(entity, "_Node__node_name", "") or ""
        )
        namespace = resolve_substitutions(
            context, getattr(entity, "_Node__node_namespace", "") or ""
        )
        package = resolve_substitutions(
            context, getattr(entity, "_Node__package", "") or ""
        )
        namespace = namespace if namespace else current_namespace
        for c in containers:
            if (
                c.executable == executable
                and c.name == name
                and c.namespace == namespace
                and c.package == package
            ):
                return True
    return False


def main():
    now = time.time()
    if len(sys.argv) < 2:
        print("provide a launch file silly")
        sys.exit(1)

    launch_file = sys.argv[1]
    os.chdir(os.path.dirname(os.path.abspath(launch_file)))

    context = LaunchContext()
    source = AnyLaunchDescriptionSource(launch_file)
    try:
        ld = source.get_launch_description(context=context)
    except Exception as e:
        print(f"{RED}Failed to load launch description: {e}{RESET}")
        sys.exit(1)
    ld.visit(context)

    nodes: List[NodeInfo] = []
    composable_nodes: List[ComposableNodeInfo] = []
    containers: List[ContainerInfo] = []

    recursively_extract_entities(
        ld.entities, context, nodes, composable_nodes, containers
    )

    nodes_dict = [asdict(n) for n in nodes]
    composable_nodes_dict = [asdict(cn) for cn in composable_nodes]
    containers_dict = [asdict(c) for c in containers]

    for n in nodes_dict:
        print(GREEN)
        print("namespace:", n["namespace"])
        print("name:", n["name"])
        print("package:", n["package"])
        print("executable:", n["executable"])
        print("launch_arguments:")
        for k, v in n["launch_arguments"].items():
            print(f"  {k}: {v}")
        print(f"{YELLOW}------")
    execution_time = time.time() - now
    print(f"{RED} Execution time: {execution_time:.2f} seconds")


if __name__ == "__main__":
    main()
