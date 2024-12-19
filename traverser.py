#!/usr/bin/env python3

import sys
import os
from dataclasses import dataclass
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.utilities import perform_substitutions
from launch.substitutions import LaunchConfiguration
from launch.substitutions.substitution_failure import SubstitutionFailure

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

    def to_dict(self):
        return {
            "package": self.package,
            "executable": self.executable,
            "name": self.name,
            "namespace": self.namespace,
        }


@dataclass(frozen=True)
class ComposableNodeInfo:
    package: str
    plugin: str
    name: str


@dataclass(frozen=True)
class ContainerInfo:
    package: str
    executable: str
    name: str
    namespace: str


def resolve_substitutions(context, value):
    # If the value is a list or tuple of substitutions, we attempt to perform them
    # If it's a single LaunchConfiguration or other substitution, wrap it in a list.
    if isinstance(value, (list, tuple)):
        subs_list = value
    else:
        # If it's a single substitution (like LaunchConfiguration), make it a list
        subs_list = [value]

    try:
        result = perform_substitutions(context, subs_list)
        return "".join(result)
    except SubstitutionFailure:
        # If we fail to resolve, return a string representation
        return str(value)
    except Exception as e:
        # Any other error, just return a string representation
        return str(value)


def recursively_extract_entities(
    entities, context, nodes, composable_nodes, containers
):
    for entity in entities:
        try:
            if isinstance(entity, IncludeLaunchDescription):
                included_ld = entity.launch_description_source.get_launch_description(
                    context=context
                )
                included_ld.visit(context)  # Expand includes, substitutions, etc.
                recursively_extract_entities(
                    included_ld.entities, context, nodes, composable_nodes, containers
                )
            elif isinstance(entity, GroupAction):
                sub_entities = entity.get_sub_entities()
                recursively_extract_entities(
                    sub_entities, context, nodes, composable_nodes, containers
                )
            else:
                if not already_found(entity, nodes, composable_nodes, containers):
                    if isinstance(entity, Node):
                        # Attempt to perform substitutions
                        try:
                            entity._perform_substitutions(context)
                        except Exception as e:
                            print(
                                f"{RED}Warning: Could not fully expand node substitutions: {e}{RESET}"
                            )

                        # Now try to retrieve final resolved values
                        # If they are not expanded, try resolving directly.

                        package = resolve_substitutions(
                            context, getattr(entity, "_Node__package", "") or ""
                        )
                        executable = resolve_substitutions(
                            context, getattr(entity, "_Node__node_executable", "") or ""
                        )

                        # For name and namespace, try expanded attributes first:
                        node_name = getattr(entity, "_Node__expanded_node_name", None)
                        node_namespace = getattr(
                            entity, "_Node__expanded_node_namespace", None
                        )

                        # If still None, try original attributes
                        if node_name is None:
                            node_name = resolve_substitutions(
                                context, getattr(entity, "_Node__node_name", "")
                            )
                        else:
                            node_name = resolve_substitutions(context, node_name)

                        if node_namespace is None:
                            node_namespace = resolve_substitutions(
                                context, getattr(entity, "_Node__node_namespace", "")
                            )
                        else:
                            node_namespace = resolve_substitutions(
                                context, node_namespace
                            )

                        # Clean double slashes if namespace is root
                        full_name = f"{node_namespace}/{node_name}"
                        full_name = full_name.replace("//", "/")

                        print(
                            f"Found Node: {GREEN}{executable}{RESET} "
                            f"with the full name: {BLUE}{full_name}{RESET} "
                            f"within package {YELLOW}{package}{RESET}"
                        )

                        node_info = NodeInfo(
                            package=package,
                            executable=executable,
                            name=node_name,
                            namespace=node_namespace,
                        )
                        nodes.append(node_info)

                    elif isinstance(entity, ComposableNode):
                        package = resolve_substitutions(
                            context,
                            getattr(entity, "_ComposableNode__package", "") or "",
                        )
                        plugin = resolve_substitutions(
                            context,
                            getattr(entity, "_ComposableNode__plugin", "") or "",
                        )
                        name = resolve_substitutions(
                            context, getattr(entity, "_ComposableNode__name", "") or ""
                        )

                        print(
                            f"Found ComposableNode: {GREEN}{plugin}{RESET} name: {BLUE}{name}{RESET} in package {YELLOW}{package}{RESET}"
                        )

                        cn_info = ComposableNodeInfo(
                            package=package, plugin=plugin, name=name
                        )
                        composable_nodes.append(cn_info)

                    elif isinstance(entity, ComposableNodeContainer):
                        # Try substitutions
                        try:
                            entity._perform_substitutions(context)
                        except Exception as e:
                            print(
                                f"{RED}Warning: Could not fully expand container substitutions: {e}{RESET}"
                            )

                        package = resolve_substitutions(
                            context, getattr(entity, "_Node__package", "") or ""
                        )
                        executable = resolve_substitutions(
                            context, getattr(entity, "_Node__node_executable", "") or ""
                        )
                        node_name = getattr(entity, "_Node__expanded_node_name", None)
                        node_namespace = getattr(
                            entity, "_Node__expanded_node_namespace", None
                        )

                        if node_name is None:
                            node_name = resolve_substitutions(
                                context, getattr(entity, "_Node__node_name", "")
                            )
                        else:
                            node_name = resolve_substitutions(context, node_name)

                        if node_namespace is None:
                            node_namespace = resolve_substitutions(
                                context, getattr(entity, "_Node__node_namespace", "")
                            )
                        else:
                            node_namespace = resolve_substitutions(
                                context, node_namespace
                            )

                        full_name = f"{node_namespace}/{node_name}"
                        full_name = full_name.replace("//", "/")

                        print(
                            f"Found ComposableNodeContainer: {GREEN}{executable}{RESET} "
                            f"with the full name: {BLUE}{full_name}{RESET} "
                            f"within package {YELLOW}{package}{RESET}"
                        )

                        container_info = ContainerInfo(
                            package=package,
                            executable=executable,
                            name=node_name,
                            namespace=node_namespace,
                        )
                        containers.append(container_info)

                if hasattr(entity, "describe_sub_entities"):
                    sub_entities = entity.describe_sub_entities()
                    if sub_entities:
                        recursively_extract_entities(
                            sub_entities, context, nodes, composable_nodes, containers
                        )
        except LookupError:
            if isinstance(entity, Node):
                resolved_package = resolve_substitutions(context, entity._Node__package)
                print(f"{RED}package could not be found: {resolved_package}{RESET}")
        except Exception as e:
            print(f"{RED}Error while extracting entities: {e}{RESET}")
            continue


def already_found(entity, nodes, composable_nodes, containers):
    # Since we now store dataclasses, we cannot directly check if entity is in the list.
    # The logic here is that we only call this before adding a new entity's info.
    # If you need to prevent duplicates based on resolved info, you can implement a check.
    return False


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 extract_entities.py <launch_file>")
        sys.exit(1)

    launch_file = sys.argv[1]
    os.chdir(os.path.dirname(launch_file))

    context = LaunchContext()
    source = AnyLaunchDescriptionSource(launch_file)
    ld = source.get_launch_description(context=context)
    ld.visit(context)

    nodes = []
    composable_nodes = []
    containers = []

    recursively_extract_entities(
        ld.entities, context, nodes, composable_nodes, containers
    )

    print(YELLOW + "Summary:" + RESET)
    print("Nodes:")
    for n in nodes:
        print(" ", n)
    print("ComposableNodes:")
    for cn in composable_nodes:
        print(" ", cn)
    print("Containers:")
    for c in containers:
        print(" ", c)

    a = NodeInfo(package="a", executable="b", name="c", namespace="d")
    a.to_dict()
    print(a)


if __name__ == "__main__":
    main()
