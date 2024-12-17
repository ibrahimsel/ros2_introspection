import inspect
import rclpy
from launch import LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    GroupAction,
)
from launch_ros.actions import Node as LaunchNode
from launch.utilities import perform_substitutions, normalize_to_list_of_substitutions
from launch_ros.substitutions import ExecutableInPackage
import traceback
from ament_index_python.packages import PackageNotFoundError
from launch.substitutions import SubstitutionFailure
from launch.actions.opaque_function import OpaqueFunction
from pprint import pprint

try:
    from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
except ImportError:
    XMLLaunchDescriptionSource = None

BLUE = "\033[94m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
RESET = "\033[0m"
RED = "\033[91m"


def debug_print(msg, color=BLUE):
    print(f"{color}[DEBUG]{RESET} {msg}")


def generic_action_to_dict(action):
    attributes = {}
    for name, value in inspect.getmembers(action):
        if not name.startswith("_") and not inspect.isroutine(value):
            attributes[name] = value
    return {"type": action.__class__.__name__, "attributes": attributes}


def include_to_dict(include_action):
    source = include_action.launch_description_source
    source_file = getattr(source, "_filename", None)
    return {
        "type": "IncludeLaunchDescription",
        "source_file": source_file,
    }


def execute_process_to_dict(proc_action):
    return {
        "type": "ExecuteProcess",
        "cmd": proc_action.cmd,
        "cwd": proc_action.cwd,
        "env": proc_action.env,
    }


def recursively_collect_actions(entities, context=None):
    from launch import LaunchContext

    if context is None:
        context = LaunchContext()
    all_actions = []
    for entity in entities:
        if isinstance(entity, IncludeLaunchDescription):
            debug_print("Found IncludeLaunchDescription, recursing.")
            try:
                included_ld = entity.launch_description_source.get_launch_description(
                    context
                )
                all_actions.extend(
                    recursively_collect_actions(included_ld.entities, context)
                )
            except PackageNotFoundError as e:
                print(f"{YELLOW}[WARN]{RESET} Could not find required package: {e}")
                print(
                    f"{YELLOW}[WARN]{RESET} Skipping this included launch description."
                )
            except LookupError as e:
                print(f"{YELLOW}[WARN]{RESET} Lookup error occurred: {e}")
                print(
                    f"{YELLOW}[WARN]{RESET} Skipping this included launch description."
                )
            except SubstitutionFailure as e:
                print(f"{YELLOW}[WARN]{RESET} Substitution failure occurred: {e}")
                print(
                    f"{YELLOW}[WARN]{RESET} Skipping this included launch description."
                )
            except Exception as e:
                # Catch other unforeseen exceptions to avoid crashing the entire program.
                print(
                    f"{YELLOW}[WARN]{RESET} An unexpected error occurred while processing include:"
                )
                print(f"{YELLOW}[WARN]{RESET} {e}")
                traceback.print_exc()
                print(
                    f"{YELLOW}[WARN]{RESET} Skipping this included launch description."
                )
        elif isinstance(entity, GroupAction):
            debug_print("Found GroupAction, recursing into its sub-entities.")
            sub_entities = entity.get_sub_entities()
            try:
                all_actions.extend(recursively_collect_actions(sub_entities, context))
            except Exception as e:
                print(f"{YELLOW}[WARN]{RESET} Error while processing GroupAction: {e}")
                traceback.print_exc()
                print(f"{YELLOW}[WARN]{RESET} Skipping this group action.")
        else:
            all_actions.append(entity)
    return all_actions


def execute_declare_launch_arguments(actions, context):
    debug_print("Executing DeclareLaunchArgument actions.")
    for action in actions:
        if isinstance(action, DeclareLaunchArgument):
            debug_print(f"Declaring launch argument: {action.name}")
            try:
                action.execute(context)
            except RuntimeError as e:
                print(f"{YELLOW}[WARN]{RESET} {e}")
                print(
                    f"{YELLOW}[WARN]{RESET} Setting '{action.name}' to an empty string by default."
                )
                context.launch_configurations[action.name] = ""
            except PackageNotFoundError as e:
                print(f"{YELLOW}[WARN]{RESET} Could not find required package: {e}")
                print(
                    f"{YELLOW}[WARN]{RESET} Skipping this launch argument declaration."
                )


def node_to_dict(action, global_context):
    local_context = LaunchContext()
    local_context.launch_configurations.update(global_context.launch_configurations)

    try:
        action._perform_substitutions(local_context)
    except Exception as e:
        print(f"{YELLOW}[WARN]{RESET} Failed to perform substitutions on Node: {e}")
        return generic_action_to_dict(action)

    try:
        final_name = action._Node__final_node_name
        remappings = action._Node__expanded_remappings
        parameters = action._Node__expanded_parameter_arguments
        package = action._Node__package
        node_executable = action._Node__node_executable
        arguments = action._Node__arguments

        # Attempt to resolve the executable
        if package is not None:
            exec_substitution = ExecutableInPackage(
                package=package, executable=node_executable
            )
            try:
                resolved_exec = perform_substitutions(
                    local_context, [exec_substitution]
                )
                executable = "".join(resolved_exec)
            except SubstitutionFailure as sub_fail:
                print(
                    f"{YELLOW}[WARN]{RESET} Executable resolution failed for package='{package}', executable='{node_executable}': {sub_fail}"
                )
                print(
                    f"{YELLOW}[WARN]{RESET} Using unmodified executable name '{node_executable}' as fallback."
                )
                executable = node_executable

            package_resolved = perform_substitutions(
                local_context, normalize_to_list_of_substitutions(package)
            )
            package = "".join(package_resolved)
        else:
            try:
                resolved_exec = perform_substitutions(
                    local_context, normalize_to_list_of_substitutions(node_executable)
                )
                executable = "".join(resolved_exec)
            except SubstitutionFailure as sub_fail:
                print(
                    f"{YELLOW}[WARN]{RESET} Executable resolution failed for executable='{node_executable}': {sub_fail}"
                )
                print(
                    f"{YELLOW}[WARN]{RESET} Using unmodified executable name '{node_executable}' as fallback."
                )
                executable = node_executable

        # Resolve arguments if any
        arguments_resolved = []
        if arguments is not None:
            for arg in arguments:
                arg_res = perform_substitutions(
                    local_context, normalize_to_list_of_substitutions(arg)
                )
                arguments_resolved.append("".join(arg_res))

        debug_print(
            f"Node resolved: name={final_name} (executable={executable}), package={package}, args={arguments_resolved}",
            GREEN,
        )

        return {
            "type": "Node",
            "package": package,
            "executable": executable,
            "name": final_name,
            "remappings": remappings,
            "parameters": parameters,
            "arguments": arguments_resolved,
        }
    except AttributeError as attr_err:
        print(
            f"{YELLOW}[WARN]{RESET} Could not access expected Node attribute: {attr_err}"
        )
        return generic_action_to_dict(action)


def opaque_function_to_dict(action, global_context):
    local_context = LaunchContext()
    local_context.launch_configurations.update(global_context.launch_configurations)

    try:
        sub_actions = action.execute(local_context)
        if sub_actions:
            sub_ir = []
            for sub_action in sub_actions:
                sub_ir.append(action_to_dict(sub_action, global_context))
            return {"type": "OpaqueFunction", "subactions": sub_ir}
        else:
            return {"type": "OpaqueFunction", "info": "No sub-actions returned"}
    except KeyError as ke:
        print(
            f"{YELLOW}[WARN]{RESET} OpaqueFunction failed due to missing parameter: {ke}"
        )
        print(
            f"{YELLOW}[WARN]{RESET} Could not provide defaults. Returning generic action info."
        )
        return generic_action_to_dict(action)
    except Exception as e:
        print(f"{YELLOW}[WARN]{RESET} Failed to execute OpaqueFunction: {e}")
        traceback.print_exc()
        print(f"{YELLOW}[WARN]{RESET} Returning a generic action representation.")
        return generic_action_to_dict(action)


def action_to_dict(action, global_context):
    from launch.actions.opaque_function import OpaqueFunction

    if isinstance(action, LaunchNode):
        return node_to_dict(action, global_context)
    elif isinstance(action, IncludeLaunchDescription):
        debug_print("Processing an IncludeLaunchDescription action.")
        return include_to_dict(action)
    elif isinstance(action, ExecuteProcess):
        debug_print("Processing an ExecuteProcess action.")
        return execute_process_to_dict(action)
    elif isinstance(action, OpaqueFunction):
        debug_print("Processing an OpaqueFunction action.")
        return opaque_function_to_dict(action, global_context)
    else:
        debug_print(f"Processing a generic action of type {type(action).__name__}.")
        return generic_action_to_dict(action)


def launch_description_to_ir(launch_description):
    debug_print("Collecting all actions from launch description.")
    all_actions = recursively_collect_actions(launch_description.entities)
    global_context = LaunchContext()
    execute_declare_launch_arguments(all_actions, global_context)

    ir = {"actions": []}
    debug_print("Converting actions to IR.")
    for action in all_actions:
        ir_action = action_to_dict(action, global_context)
        ir["actions"].append(ir_action)

    debug_print("Collected actions in IR:")
    pprint(ir, width=120, indent=2)

    return ir


def environment_to_ir():
    # Get runtime environment nodes
    rclpy.init()
    node = rclpy.create_node("_ir_builder")
    node_info = node.get_node_names_and_namespaces()
    node.destroy_node()
    rclpy.shutdown()

    env_ir = {"nodes": []}
    for n, ns in node_info:
        if ns == "/":
            full_name = f"{ns}{n}"
        else:
            full_name = f"{ns}/{n}"
        env_ir["nodes"].append({"full_name": full_name, "type": "NodeRuntime"})
    return env_ir
