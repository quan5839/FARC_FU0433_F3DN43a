#!/usr/bin/env python3
import argparse
import hashlib
import json
import os
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any, Dict, List

from ci.running_process import RunningProcess


_PIO_CHECK_ENABLED = False

# Default to non-interactive mode for safety unless explicitly in interactive environment
# This prevents hanging on prompts when running in automated environments
_IS_GITHUB = True  # Default to non-interactive behavior
if os.environ.get("FASTLED_INTERACTIVE") == "true":
    _IS_GITHUB = False  # Only enable interactive mode if explicitly requested

# Set environment variable to ensure all subprocesses also run in non-interactive mode
os.environ["FASTLED_CI_NO_INTERACTIVE"] = "true"
if not _IS_GITHUB:
    os.environ.pop(
        "FASTLED_CI_NO_INTERACTIVE", None
    )  # Remove if interactive mode is enabled


def run_command(cmd: List[str], **kwargs: Any) -> None:
    """Run a command and handle errors"""
    try:
        subprocess.run(cmd, check=True, **kwargs)
    except subprocess.CalledProcessError as e:
        sys.exit(e.returncode)


def parse_args() -> argparse.Namespace:
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description="Run FastLED tests")
    parser.add_argument("--cpp", action="store_true", help="Run C++ tests only")
    parser.add_argument(
        "test", type=str, nargs="?", default=None, help="Specific C++ test to run"
    )

    # Create mutually exclusive group for compiler selection
    compiler_group = parser.add_mutually_exclusive_group()
    compiler_group.add_argument(
        "--clang", action="store_true", help="Use Clang compiler"
    )
    compiler_group.add_argument(
        "--gcc", action="store_true", help="Use GCC compiler (default on non-Windows)"
    )

    parser.add_argument(
        "--clean", action="store_true", help="Clean build before compiling"
    )
    parser.add_argument(
        "--no-interactive",
        action="store_true",
        help="Force non-interactive mode (no confirmation prompts)",
    )
    parser.add_argument(
        "--interactive",
        action="store_true",
        help="Enable interactive mode (allows confirmation prompts)",
    )
    parser.add_argument(
        "--verbose",
        "-v",
        action="store_true",
        help="Enable verbose output showing all test details",
    )
    parser.add_argument(
        "--quick", action="store_true", help="Enable quick mode with FASTLED_ALL_SRC=1"
    )
    parser.add_argument(
        "--no-stack-trace",
        action="store_true",
        help="Disable stack trace dumping on timeout",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="Enable static analysis (IWYU, clang-tidy) - auto-enables --cpp and --clang",
    )

    args = parser.parse_args()

    # Auto-enable --cpp when a specific test is provided
    if args.test and not args.cpp:
        args.cpp = True
        print(f"Auto-enabled --cpp mode for specific test: {args.test}")

    # Auto-enable --cpp and --clang when --check is provided
    if args.check:
        if not args.cpp:
            args.cpp = True
            print("Auto-enabled --cpp mode for static analysis (--check)")
        if not args.clang and not args.gcc:
            args.clang = True
            print("Auto-enabled --clang compiler for static analysis (--check)")

    # Default to Clang on Windows unless --gcc is explicitly passed
    if sys.platform == "win32" and not args.gcc and not args.clang:
        args.clang = True
        print("Windows detected: defaulting to Clang compiler (use --gcc to override)")
    elif args.gcc:
        print("Using GCC compiler")
    elif args.clang:
        print("Using Clang compiler")

    return args


def _make_pio_check_cmd() -> List[str]:
    return [
        "pio",
        "check",
        "--skip-packages",
        "--src-filters=+<src/>",
        "--severity=medium",
        "--fail-on-defect=high",
        "--flags",
        "--inline-suppr --enable=all --std=c++17",
    ]


def make_compile_uno_test_process(enable_stack_trace: bool = True) -> RunningProcess:
    """Create a process to compile the uno tests"""
    cmd = [
        "uv",
        "run",
        "ci/ci-compile.py",
        "uno",
        "--examples",
        "Blink",
        "--no-interactive",
    ]
    # shell=True wasn't working for some reason
    # cmd = cmd + ['||'] + cmd
    # return RunningProcess(cmd, echo=False, auto_run=not _IS_GITHUB, shell=True)
    return RunningProcess(
        cmd, echo=False, auto_run=not _IS_GITHUB, enable_stack_trace=enable_stack_trace
    )


def fingerprint_code_base(
    start_directory: Path, glob: str = "**/*.h,**/*.cpp,**/*.hpp"
) -> Dict[str, str]:
    """
    Create a fingerprint of the code base by hashing file contents.

    Args:
        start_directory: The root directory to start scanning from
        glob: Comma-separated list of glob patterns to match files

    Returns:
        A dictionary with hash and status
    """
    result = {
        "hash": "",
    }

    try:
        hasher = hashlib.sha256()
        patterns = glob.split(",")

        # Get all matching files
        all_files = []
        for pattern in patterns:
            pattern = pattern.strip()
            all_files.extend(sorted(start_directory.glob(pattern)))

        # Sort files for consistent ordering
        all_files.sort()

        # Process each file
        for file_path in all_files:
            if file_path.is_file():
                # Add the relative path to the hash
                rel_path = file_path.relative_to(start_directory)
                hasher.update(str(rel_path).encode("utf-8"))

                # Add the file content to the hash
                try:
                    with open(file_path, "rb") as f:
                        # Read in chunks to handle large files
                        for chunk in iter(lambda: f.read(4096), b""):
                            hasher.update(chunk)
                except Exception as e:
                    # If we can't read the file, include the error in the hash
                    hasher.update(f"ERROR:{str(e)}".encode("utf-8"))

        result["hash"] = hasher.hexdigest()
        return result
    except Exception as e:
        result["status"] = f"error: {str(e)}"
        return result


def calculate_fingerprint(root_dir: Path | None = None) -> Dict[str, str]:
    """
    Calculate the code base fingerprint.

    Args:
        root_dir: The root directory to start scanning from. If None, uses src directory.

    Returns:
        The fingerprint result dictionary
    """
    if root_dir is None:
        root_dir = Path.cwd() / "src"

    start_time = time.time()
    # Compute the fingerprint
    result = fingerprint_code_base(root_dir)
    elapsed_time = time.time() - start_time
    # Add timing information to the result
    result["elapsed_seconds"] = f"{elapsed_time:.2f}"

    return result


def main() -> None:
    try:
        # Start a watchdog timer to kill the process if it takes too long (10 minutes)
        def watchdog_timer():
            time.sleep(600)  # 10 minutes
            print("Watchdog timer expired after 10 minutes - forcing exit")
            os._exit(2)  # Exit with error code 2 to indicate timeout

        watchdog = threading.Thread(
            target=watchdog_timer, daemon=True, name="WatchdogTimer"
        )
        watchdog.start()

        args = parse_args()

        # Handle --no-interactive flag
        if args.no_interactive:
            global _IS_GITHUB
            _IS_GITHUB = True
            os.environ["FASTLED_CI_NO_INTERACTIVE"] = "true"
            os.environ["GITHUB_ACTIONS"] = (
                "true"  # This ensures all subprocess also run in non-interactive mode
            )

        # Handle --interactive flag
        if args.interactive:
            _IS_GITHUB = False
            os.environ.pop("FASTLED_CI_NO_INTERACTIVE", None)
            os.environ.pop("GITHUB_ACTIONS", None)

        # Handle --quick flag
        if args.quick:
            os.environ["FASTLED_ALL_SRC"] = "1"
            print("Quick mode enabled. FASTLED_ALL_SRC=1")

        # Handle stack trace control
        enable_stack_trace = not args.no_stack_trace
        if enable_stack_trace:
            print("Stack trace dumping enabled for test timeouts")
        else:
            print("Stack trace dumping disabled for test timeouts")

        # Validate conflicting arguments
        if args.no_interactive and args.interactive:
            print(
                "Error: --interactive and --no-interactive cannot be used together",
                file=sys.stderr,
            )
            sys.exit(1)

        # Change to script directory
        os.chdir(Path(__file__).parent)

        cache_dir = Path(".cache")
        cache_dir.mkdir(exist_ok=True)
        fingerprint_file = cache_dir / "fingerprint.json"

        def write_fingerprint(fingerprint: Dict[str, str]) -> None:
            with open(fingerprint_file, "w") as f:
                json.dump(fingerprint, f, indent=2)

        def read_fingerprint() -> Dict[str, str] | None:
            if fingerprint_file.exists():
                with open(fingerprint_file, "r") as f:
                    try:
                        return json.load(f)
                    except json.JSONDecodeError:
                        print("Invalid fingerprint file. Recalculating...")
            return None

        prev_fingerprint: dict[str, str] | None = read_fingerprint()
        # Calculate fingerprint
        fingerprint_data = calculate_fingerprint()
        src_code_change: bool
        if prev_fingerprint is None:
            src_code_change = True
        else:
            try:
                src_code_change = fingerprint_data["hash"] != prev_fingerprint["hash"]
            except KeyError:
                print("Invalid fingerprint file. Recalculating...")
                src_code_change = True
        # print(f"Fingerprint: {fingerprint_result['hash']}")

        # Create .cache directory if it doesn't exist

        # Save the fingerprint to a file as JSON

        write_fingerprint(fingerprint_data)

        cmd_list = ["uv", "run", "ci/cpp_test_run.py"]

        if args.clang:
            cmd_list.append("--clang")

        if args.test:
            cmd_list.append("--test")
            cmd_list.append(args.test)
        if args.clean:
            cmd_list.append("--clean")
        if args.verbose:
            cmd_list.append("--verbose")
        if args.check:
            cmd_list.append("--check")

        cmd_str_cpp = subprocess.list2cmdline(cmd_list)

        if args.cpp:
            # Compile and run C++ tests
            start_time = time.time()

            if args.test:
                # Run specific C++ test
                proc = RunningProcess(
                    cmd_str_cpp, enable_stack_trace=enable_stack_trace
                )
                proc.wait()
                if proc.returncode != 0:
                    print(f"Command failed: {proc.command}")
                    sys.exit(proc.returncode)
            else:
                # Run all C++ tests
                proc = RunningProcess(
                    cmd_str_cpp, enable_stack_trace=enable_stack_trace
                )
                proc.wait()
                if proc.returncode != 0:
                    print(f"Command failed: {proc.command}")
                    sys.exit(proc.returncode)

            print(f"Time elapsed: {time.time() - start_time:.2f}s")
            return

        cmd_list = _make_pio_check_cmd()
        if not _PIO_CHECK_ENABLED:
            cmd_list = ["echo", "pio check is disabled"]

        cmd_str = subprocess.list2cmdline(cmd_list)

        print(f"Running command (in the background): {cmd_str}")
        pio_process = RunningProcess(
            cmd_str,
            echo=False,
            auto_run=not _IS_GITHUB,
            enable_stack_trace=enable_stack_trace,
        )
        cpp_test_proc = RunningProcess(
            cmd_str_cpp, enable_stack_trace=enable_stack_trace
        )
        compile_native_proc = RunningProcess(
            "uv run ci/ci-compile-native.py",
            echo=False,
            auto_run=not _IS_GITHUB,
            enable_stack_trace=enable_stack_trace,
        )
        pytest_proc = RunningProcess(
            "uv run pytest -s ci/tests -xvs --durations=0",
            echo=True,
            auto_run=not _IS_GITHUB,
            enable_stack_trace=enable_stack_trace,
        )

        tests = [
            cpp_test_proc,
            compile_native_proc,
            pytest_proc,
            pio_process,
        ]
        if src_code_change:
            print("Source code changed, running uno tests")
            tests += [make_compile_uno_test_process(enable_stack_trace)]

        is_first = True
        for test in tests:
            was_first = is_first
            is_first = False
            sys.stdout.flush()
            if not test.auto_run:
                test.run()
            print(f"Waiting for command: {test.command}")
            # make a thread that will say waiting for test {test} to finish...<seconds>
            # and then kill the test if it takes too long (> 120 seconds)
            event_stopped = threading.Event()

            def _runner() -> None:
                start_time = time.time()
                while not event_stopped.wait(1):
                    curr_time = time.time()
                    seconds = int(curr_time - start_time)
                    if (
                        not was_first
                    ):  # skip printing for the first test since it echo's out.
                        print(
                            f"Waiting for command: {test.command} to finish...{seconds} seconds"
                        )

            runner_thread = threading.Thread(target=_runner, daemon=True)
            runner_thread.start()
            test.wait()
            event_stopped.set()
            runner_thread.join(timeout=1)
            if not test.echo:
                for line in test.stdout.splitlines():
                    print(line)
            if test.returncode != 0:
                [t.kill() for t in tests]
                print(
                    f"\nCommand failed: {test.command} with return code {test.returncode}"
                )
                sys.exit(test.returncode)

        print("All tests passed")

        # Launch a force exit daemon thread that waits for 1 second until invoking os._exit(0)
        def force_exit():
            time.sleep(1)
            print("Force exit daemon thread invoked")
            os._exit(1)

        daemon_thread = threading.Thread(
            target=force_exit, daemon=True, name="ForceExitDaemon"
        )
        daemon_thread.start()
        sys.exit(0)
    except KeyboardInterrupt:
        sys.exit(130)  # Standard Unix practice: 128 + SIGINT's signal number (2)


if __name__ == "__main__":
    main()
