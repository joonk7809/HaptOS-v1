#!/usr/bin/env python3
"""
HaptOS Project Initialization and Verification Script

Checks project structure, dependencies, and runs basic validation tests.
"""

import sys
import os
from pathlib import Path
import subprocess

# ANSI color codes
GREEN = '\033[92m'
YELLOW = '\033[93m'
RED = '\033[91m'
BLUE = '\033[94m'
RESET = '\033[0m'
BOLD = '\033[1m'

def print_header(text):
    print(f"\n{BOLD}{BLUE}{'='*70}{RESET}")
    print(f"{BOLD}{BLUE}{text.center(70)}{RESET}")
    print(f"{BOLD}{BLUE}{'='*70}{RESET}\n")

def print_success(text):
    print(f"{GREEN}✓{RESET} {text}")

def print_warning(text):
    print(f"{YELLOW}⚠{RESET} {text}")

def print_error(text):
    print(f"{RED}✗{RESET} {text}")

def print_info(text):
    print(f"{BLUE}ℹ{RESET} {text}")

def check_python_version():
    """Check Python version is 3.8+"""
    print_header("Python Environment")

    version = sys.version_info
    version_str = f"{version.major}.{version.minor}.{version.micro}"

    if version.major >= 3 and version.minor >= 8:
        print_success(f"Python version: {version_str}")
        return True
    else:
        print_error(f"Python version {version_str} - Requires 3.8+")
        return False

def check_directory_structure():
    """Verify project directory structure"""
    print_header("Directory Structure")

    required_dirs = [
        'src/core',
        'src/routing',
        'src/physics',
        'src/inference',
        'src/audio',
        'src/scenarios',
        'tests/phase1',
        'tests/integration',
        'scripts/demos',
        'scripts/tools',
        'docs/guides',
        'docs/setup',
        'assets/hand_models',
        'config',
        'models/checkpoints',
        'firmware'
    ]

    all_exist = True
    for dir_path in required_dirs:
        full_path = Path(dir_path)
        if full_path.exists():
            print_success(f"{dir_path}")
        else:
            print_error(f"{dir_path} - MISSING")
            all_exist = False

    return all_exist

def check_required_files():
    """Check for essential project files"""
    print_header("Required Files")

    required_files = [
        'README.md',
        'ARCHITECTURE.md',
        'README_STRUCTURE.md',
        'src/core/schemas.py',
        'src/routing/somatotopic_router.py',
        'src/physics/multi_contact_engine.py',
        'tests/phase1/test_schemas.py',
        'tests/phase1/test_router.py',
        'tests/phase1/test_integration.py',
        'docs/PHASE1_PROGRESS.md',
        'docs/FILE_STRUCTURE.md'
    ]

    all_exist = True
    for file_path in required_files:
        full_path = Path(file_path)
        if full_path.exists():
            size = full_path.stat().st_size
            print_success(f"{file_path} ({size:,} bytes)")
        else:
            print_error(f"{file_path} - MISSING")
            all_exist = False

    return all_exist

def check_python_packages():
    """Check required Python packages"""
    print_header("Python Dependencies")

    required_packages = [
        'numpy',
        'mujoco',
        'torch',
        'pytest',
        'pyyaml'
    ]

    all_installed = True
    for package in required_packages:
        try:
            if package == 'mujoco':
                import mujoco
                print_success(f"{package} - version {mujoco.__version__}")
            elif package == 'torch':
                import torch
                print_success(f"{package} - version {torch.__version__}")
            elif package == 'numpy':
                import numpy
                print_success(f"{package} - version {numpy.__version__}")
            elif package == 'pytest':
                import pytest
                print_success(f"{package} - version {pytest.__version__}")
            elif package == 'pyyaml':
                import yaml
                print_success(f"{package} (yaml)")
            else:
                __import__(package)
                print_success(f"{package}")
        except ImportError:
            print_error(f"{package} - NOT INSTALLED")
            all_installed = False

    return all_installed

def check_hand_models():
    """Check MuJoCo hand model files"""
    print_header("Hand Models")

    model_files = [
        'assets/hand_models/detailed_hand.xml',
        'assets/hand_models/simple_hand.xml'
    ]

    all_exist = True
    for model_path in model_files:
        full_path = Path(model_path)
        if full_path.exists():
            size = full_path.stat().st_size
            print_success(f"{model_path} ({size:,} bytes)")
        else:
            print_warning(f"{model_path} - Not found (optional)")

    return True  # Models are optional

def check_trained_models():
    """Check for trained neural network models"""
    print_header("Trained Models")

    model_files = [
        'models/checkpoints/nn_v0_best.pt',
        'models/checkpoints/nn_v1_best.pt'
    ]

    for model_path in model_files:
        full_path = Path(model_path)
        if full_path.exists():
            size = full_path.stat().st_size
            print_success(f"{model_path} ({size:,} bytes)")
        else:
            print_warning(f"{model_path} - Not found (optional for Phase 1)")

    return True  # Models are optional for Phase 1

def run_unit_tests():
    """Run Phase 1 unit tests"""
    print_header("Unit Tests (Phase 1)")

    try:
        result = subprocess.run(
            ['python', '-m', 'pytest', 'tests/phase1/', '-v', '--tb=short'],
            capture_output=True,
            text=True,
            timeout=60
        )

        if result.returncode == 0:
            # Count passed tests
            output = result.stdout
            if 'passed' in output:
                print_success("All Phase 1 tests passed!")
                # Show summary line
                for line in output.split('\n'):
                    if 'passed' in line and '==' in line:
                        print_info(f"  {line.strip()}")
            return True
        else:
            print_error("Some tests failed")
            print(result.stdout[-500:])  # Last 500 chars
            return False
    except subprocess.TimeoutExpired:
        print_error("Tests timed out")
        return False
    except Exception as e:
        print_error(f"Could not run tests: {e}")
        return False

def check_imports():
    """Test critical imports"""
    print_header("Module Imports")

    imports_to_test = [
        ('src.core.schemas', ['ContactPatch', 'FilteredContact', 'CueParams']),
        ('src.routing.somatotopic_router', ['SomatotopicRouter', 'Homunculus']),
        ('src.physics.multi_contact_engine', ['MultiContactEngine']),
    ]

    all_ok = True
    for module_name, classes in imports_to_test:
        try:
            module = __import__(module_name, fromlist=classes)
            for class_name in classes:
                if hasattr(module, class_name):
                    print_success(f"{module_name}.{class_name}")
                else:
                    print_error(f"{module_name}.{class_name} - NOT FOUND")
                    all_ok = False
        except ImportError as e:
            print_error(f"{module_name} - IMPORT ERROR: {e}")
            all_ok = False

    return all_ok

def create_init_summary():
    """Create initialization summary"""
    summary = {
        'python': False,
        'structure': False,
        'files': False,
        'packages': False,
        'imports': False,
        'tests': False
    }

    summary['python'] = check_python_version()
    summary['structure'] = check_directory_structure()
    summary['files'] = check_required_files()
    summary['packages'] = check_python_packages()
    check_hand_models()  # Optional
    check_trained_models()  # Optional
    summary['imports'] = check_imports()
    summary['tests'] = run_unit_tests()

    return summary

def print_final_summary(summary):
    """Print final initialization summary"""
    print_header("Initialization Summary")

    all_passed = all(summary.values())

    status_map = {
        'python': 'Python Environment',
        'structure': 'Directory Structure',
        'files': 'Required Files',
        'packages': 'Python Dependencies',
        'imports': 'Module Imports',
        'tests': 'Unit Tests'
    }

    for key, label in status_map.items():
        if summary[key]:
            print_success(f"{label}")
        else:
            print_error(f"{label}")

    print(f"\n{BOLD}{'='*70}{RESET}\n")

    if all_passed:
        print(f"{GREEN}{BOLD}🎉 HaptOS is ready to go!{RESET}\n")
        print("Next steps:")
        print("  • Run a demo: python scripts/demos/simple_haptics_test.py")
        print("  • Read docs: docs/guides/START_HERE.md")
        print("  • Check architecture: ARCHITECTURE.md")
    else:
        print(f"{YELLOW}{BOLD}⚠ Some checks failed{RESET}\n")
        print("Please fix the issues above before proceeding.")
        print("See README_STRUCTURE.md for guidance.")

    print(f"\n{BOLD}{'='*70}{RESET}\n")

    return all_passed

def main():
    """Main initialization function"""
    print(f"\n{BOLD}{BLUE}")
    print("  _   _             _    ___  ____  ")
    print(" | | | | __ _ _ __ | |_ / _ \\/ ___| ")
    print(" | |_| |/ _` | '_ \\| __| | | \\___ \\ ")
    print(" |  _  | (_| | |_) | |_| |_| |___) |")
    print(" |_| |_|\\__,_| .__/ \\__|\\___/|____/ ")
    print("             |_|                     ")
    print(f"{RESET}")
    print(f"{BOLD}HaptOS Platform - Initialization & Verification{RESET}\n")

    # Change to project root
    project_root = Path(__file__).parent
    os.chdir(project_root)
    print_info(f"Project root: {project_root}\n")

    # Run all checks
    summary = create_init_summary()

    # Print summary
    success = print_final_summary(summary)

    # Exit with appropriate code
    sys.exit(0 if success else 1)

if __name__ == '__main__':
    main()
