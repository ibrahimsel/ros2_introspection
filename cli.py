import sys
import time
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchContext
from ir_builder import (
    launch_description_to_ir,
    environment_to_ir,
)
import traceback
from comparer import compare_environment_and_launch, print_comparison_result

try:
    from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
except ImportError:
    XMLLaunchDescriptionSource = None
    # If not installed or available, handle accordingly


def load_launch_description(file_path):
    # Detect extension
    ext = os.path.splitext(file_path)[1].lower()

    if ext == ".py":
        source = PythonLaunchDescriptionSource(file_path)
    elif ext == ".xml":
        if XMLLaunchDescriptionSource is None:
            print(
                "[WARN] XMLLaunchDescriptionSource not available. Improper launch library installation."
            )
            return None
        source = XMLLaunchDescriptionSource(file_path)
    else:
        print(f"[WARN] Unrecognized launch file extension '{ext}'.")
        return None

    context = LaunchContext()
    try:
        ld = source.get_launch_description(context)
        return ld
    except SyntaxError as e:
        print(f"[ERROR] SyntaxError while parsing {file_path}: {e}")
    except Exception as e:
        print(f"[ERROR] Failed to load launch description from {file_path}: {e}")

    return None


def main():
    now = time.time()
    try:
        if len(sys.argv) != 2:
            print("Usage: python cli.py <launch_file>")
            sys.exit(1)

        file_path = sys.argv[1]
        os.chdir(file_path.rsplit("/", 1)[0])
        if not os.path.exists(file_path):
            print("Provided file does not exist.")
            sys.exit(1)

        ld = load_launch_description(file_path)
        if ld is None:
            print("[WARN] No LaunchDescription could be loaded.")
            sys.exit(1)
        launch_ir = launch_description_to_ir(ld)
        env_ir = environment_to_ir()

        print("[DEBUG] Comparing environment and launch IR.")
        comparison_result = compare_environment_and_launch(env_ir, launch_ir)
        print(f"Execution time: {time.time() - now:.2f}s")
        print_comparison_result(comparison_result)
    except Exception:

        print(f"Process did NOT finish cleanly: {traceback.print_exc()}")
        print(f"Execution time: {time.time() - now:.2f}s")
        sys.exit(1)


if __name__ == "__main__":
    main()
