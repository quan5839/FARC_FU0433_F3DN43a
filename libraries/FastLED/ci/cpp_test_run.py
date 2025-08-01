import argparse
import os
import re
import subprocess
import sys
import tempfile
from dataclasses import dataclass

from ci.paths import PROJECT_ROOT


# Configure console for UTF-8 output on Windows
if os.name == "nt":  # Windows
    # Try to set console to UTF-8 mode
    try:
        # Set stdout and stderr to UTF-8 encoding
        # Note: reconfigure() was added in Python 3.7
        if hasattr(sys.stdout, "reconfigure") and callable(
            getattr(sys.stdout, "reconfigure", None)
        ):
            sys.stdout.reconfigure(encoding="utf-8", errors="replace")  # type: ignore[attr-defined]
        if hasattr(sys.stderr, "reconfigure") and callable(
            getattr(sys.stderr, "reconfigure", None)
        ):
            sys.stderr.reconfigure(encoding="utf-8", errors="replace")  # type: ignore[attr-defined]
    except (AttributeError, OSError):
        # Fallback for older Python versions or if reconfigure fails
        pass


# Global verbose flag
_VERBOSE = False


@dataclass
class FailedTest:
    name: str
    return_code: int
    stdout: str


def check_iwyu_available() -> bool:
    """Check if include-what-you-use is available in the system"""
    try:
        result = subprocess.run(
            ["include-what-you-use", "--version"],
            capture_output=True,
            text=True,
            timeout=10,
        )
        return result.returncode == 0
    except (
        subprocess.CalledProcessError,
        FileNotFoundError,
        subprocess.TimeoutExpired,
    ):
        return False


def run_command(command, use_gdb=False) -> tuple[int, str]:
    captured_lines = []
    if use_gdb:
        with tempfile.NamedTemporaryFile(mode="w+", delete=False) as gdb_script:
            gdb_script.write("set pagination off\n")
            gdb_script.write("run\n")
            gdb_script.write("bt full\n")
            gdb_script.write("info registers\n")
            gdb_script.write("x/16i $pc\n")
            gdb_script.write("thread apply all bt full\n")
            gdb_script.write("quit\n")

        gdb_command = (
            f"gdb -return-child-result -batch -x {gdb_script.name} --args {command}"
        )
        process = subprocess.Popen(
            gdb_command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,  # Merge stderr into stdout
            shell=True,
            text=False,
        )
        assert process.stdout is not None
        # Stream and capture output
        while True:
            line_bytes = process.stdout.readline()
            line = line_bytes.decode("utf-8", errors="ignore")
            if not line and process.poll() is not None:
                break
            if line:
                captured_lines.append(line.rstrip())
                if _VERBOSE:
                    try:
                        print(line, end="")  # Only print in real-time if verbose
                    except UnicodeEncodeError:
                        # Fallback: replace problematic characters
                        print(
                            line.encode("utf-8", errors="replace").decode(
                                "utf-8", errors="replace"
                            ),
                            end="",
                        )

        os.unlink(gdb_script.name)
        output = "\n".join(captured_lines)
        return process.returncode, output
    else:
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,  # Merge stderr into stdout
            shell=True,
            text=False,
        )
        assert process.stdout is not None
        # Stream and capture output
        while True:
            line_bytes = process.stdout.readline()
            line = line_bytes.decode("utf-8", errors="ignore")
            if not line and process.poll() is not None:
                break
            if line:
                captured_lines.append(line.rstrip())
                if _VERBOSE:
                    try:
                        print(line, end="")  # Only print in real-time if verbose
                    except UnicodeEncodeError:
                        # Fallback: replace problematic characters
                        print(
                            line.encode("utf-8", errors="replace").decode(
                                "utf-8", errors="replace"
                            ),
                            end="",
                        )

        output = "\n".join(captured_lines)
        return process.returncode, output


def compile_tests(
    clean: bool = False, unknown_args: list[str] = [], specific_test: str | None = None
) -> None:
    os.chdir(str(PROJECT_ROOT))
    if _VERBOSE:
        print("Compiling tests...")
    command = ["uv", "run", "ci/cpp_test_compile.py"]
    if clean:
        command.append("--clean")
    if specific_test:
        command.extend(["--test", specific_test])
    command.extend(unknown_args)
    return_code, output = run_command(" ".join(command))
    if return_code != 0:
        print("Compilation failed:")
        print(output)  # Always show output on failure
        sys.exit(1)
    print("Compilation successful.")

    # Check if static analysis was requested and warn about IWYU availability
    if "--check" in unknown_args:
        if not check_iwyu_available():
            print(
                "⚠️  WARNING: IWYU (include-what-you-use) not found - static analysis will be limited"
            )
            print("   Install IWYU to enable include analysis:")
            print("     Windows: Install via LLVM or build from source")
            print("     Ubuntu/Debian: sudo apt install iwyu")
            print("     macOS: brew install include-what-you-use")
            print("     Or build from source: https://include-what-you-use.org/")


def run_tests(specific_test: str | None = None) -> None:
    test_dir = os.path.join("tests", ".build", "bin")
    if not os.path.exists(test_dir):
        print(f"Test directory not found: {test_dir}")
        sys.exit(1)

    print("Running tests...")
    failed_tests: list[FailedTest] = []
    files = os.listdir(test_dir)
    # filter out all pdb files (windows) and only keep test_ executables
    files = [f for f in files if not f.endswith(".pdb") and f.startswith("test_")]

    # If specific test is specified, filter for just that test
    if specific_test:
        # Check if the test name already starts with "test_" prefix
        if specific_test.startswith("test_"):
            test_name = specific_test
        else:
            test_name = f"test_{specific_test}"

        if sys.platform == "win32":
            test_name += ".exe"
        files = [f for f in files if f == test_name]
        if not files:
            print(f"Test {test_name} not found in {test_dir}")
            sys.exit(1)
    for test_file in files:
        test_path = os.path.join(test_dir, test_file)
        if os.path.isfile(test_path) and os.access(test_path, os.X_OK):
            print(f"Running test: {test_file}")
            return_code, stdout = run_command(test_path)

            output = stdout
            failure_pattern = re.compile(r"Test .+ failed with return code (\d+)")
            failure_match = failure_pattern.search(output)
            is_crash = failure_match is not None

            if is_crash:
                print("Test crashed. Re-running with GDB to get stack trace...")
                _, gdb_stdout = run_command(test_path, use_gdb=True)
                stdout += "\n--- GDB Output ---\n" + gdb_stdout

                # Extract crash information
                crash_info = extract_crash_info(gdb_stdout)
                print(f"Crash occurred at: {crash_info.file}:{crash_info.line}")
                print(f"Cause: {crash_info.cause}")
                print(f"Stack: {crash_info.stack}")

            print("Test output:")
            print(stdout)
            if return_code == 0:
                print("Test passed")
            elif is_crash:
                if failure_match:
                    print(
                        f"Test {test_file} crashed with return code {failure_match.group(1)}"
                    )
                else:
                    print(f"Test {test_file} crashed with return code {return_code}")
                # Always show crash output, even in non-verbose mode
                print("Test output:")
                print(stdout)
            else:
                print(f"Test {test_file} failed with return code {return_code}")

                print("Test output:")
                print(stdout)  # Show output on failure even in non-verbose mode

            print("-" * 40)
            if return_code != 0:
                failed_tests.append(FailedTest(test_file, return_code, stdout))
    if failed_tests:
        print("Failed tests summary:")
        for failed_test in failed_tests:
            print(
                f"Test {failed_test.name} failed with return code {failed_test.return_code}"
            )
            # Always show output on failure
            print("Output:")
            # Show indented output for better readability
            for line in failed_test.stdout.splitlines():
                print(f"  {line}")
            print()  # Add spacing between failed tests
        tests_failed = len(failed_tests)
        failed_test_names = [test.name for test in failed_tests]
        print(
            f"{tests_failed} test{'s' if tests_failed != 1 else ''} failed: {', '.join(failed_test_names)}"
        )
        sys.exit(1)
    if _VERBOSE:
        print("All tests passed.")


@dataclass
class CrashInfo:
    cause: str = "Unknown"
    stack: str = "Unknown"
    file: str = "Unknown"
    line: str = "Unknown"


def extract_crash_info(gdb_output: str) -> CrashInfo:
    lines = gdb_output.split("\n")
    crash_info = CrashInfo()

    try:
        for i, line in enumerate(lines):
            if line.startswith("Program received signal"):
                try:
                    crash_info.cause = line.split(":", 1)[1].strip()
                except IndexError:
                    crash_info.cause = line.strip()
            elif line.startswith("#0"):
                crash_info.stack = line
                for j in range(i, len(lines)):
                    if "at" in lines[j]:
                        try:
                            _, location = lines[j].split("at", 1)
                            location = location.strip()
                            if ":" in location:
                                crash_info.file, crash_info.line = location.rsplit(
                                    ":", 1
                                )
                            else:
                                crash_info.file = location
                        except ValueError:
                            pass  # If split fails, we keep the default values
                        break
                break
    except Exception as e:
        print(f"Error parsing GDB output: {e}")

    return crash_info


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Compile and run C++ tests")
    parser.add_argument(
        "--compile-only",
        action="store_true",
        help="Only compile the tests without running them",
    )
    parser.add_argument(
        "--run-only",
        action="store_true",
        help="Only run the tests without compiling them",
    )
    parser.add_argument(
        "--only-run-failed-test",
        action="store_true",
        help="Only run the tests that failed in the previous run",
    )
    parser.add_argument(
        "--clean", action="store_true", help="Clean build before compiling"
    )
    parser.add_argument(
        "--test",
        help="Specific test to run (without extension)",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Enable verbose output",
    )

    # Create mutually exclusive group for compiler selection
    compiler_group = parser.add_mutually_exclusive_group()
    compiler_group.add_argument(
        "--clang",
        help="Use Clang compiler",
        action="store_true",
    )
    compiler_group.add_argument(
        "--gcc",
        help="Use GCC compiler (default on non-Windows)",
        action="store_true",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="Enable static analysis (IWYU, clang-tidy)",
    )

    args, unknown = parser.parse_known_args()
    args.unknown = unknown

    return args


def main() -> None:
    args = parse_args()

    # Set global verbose flag
    global _VERBOSE
    _VERBOSE = args.verbose

    run_only = args.run_only
    compile_only = args.compile_only
    specific_test = args.test
    only_run_failed_test = args.only_run_failed_test
    use_clang = args.clang
    # use_gcc = args.gcc

    if not run_only:
        passthrough_args = args.unknown
        if use_clang:
            passthrough_args.append("--use-clang")
        if args.check:
            passthrough_args.append("--check")
        # Note: --gcc is handled by not passing --use-clang (GCC is the default in cpp_test_compile.py)
        compile_tests(
            clean=args.clean, unknown_args=passthrough_args, specific_test=specific_test
        )

    if not compile_only:
        if specific_test:
            run_tests(specific_test)
        else:
            cmd = "ctest --test-dir tests/.build"
            if not _VERBOSE:
                # Show progress, and output on failure by default
                cmd += " --progress --output-on-failure"
            else:
                # Full verbose mode when explicitly requested
                cmd += " --verbose --progress --output-on-failure"
            if only_run_failed_test:
                cmd += " --rerun-failed"
            rtn, stdout = run_command(cmd)
            if rtn != 0:
                print("Failed tests:")
                print(stdout)
                sys.exit(1)


if __name__ == "__main__":
    main()
