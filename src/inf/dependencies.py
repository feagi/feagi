
# Copyright 2016-2024 Neuraville Inc. Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

import pkg_resources
import sys


def validate_requirements(requirements_file='requirements.txt'):
    """
    Validates that all packages listed in the given requirements file match their installed versions.

    :param requirements_file: The path to the requirements file. Default is 'requirements.txt'.
    :raises SystemExit: If any package does not match the required version or is not installed.
    """
    with open(requirements_file, 'r') as file:
        requirements = file.readlines()

    mismatched_packages = []

    for requirement in requirements:
        try:
            # Parse the requirement line
            req = pkg_resources.Requirement.parse(requirement.strip())

            # Get the installed version of the package
            installed_version = pkg_resources.get_distribution(req.name).version

            # Compare installed version with the required version
            if installed_version not in req:
                mismatched_packages.append((req.name, req.specs, installed_version))

        except pkg_resources.DistributionNotFound:
            print(f"Package {req.name} is not installed.")
            sys.exit(1)
        except Exception as e:
            print(f"Error processing {requirement}: {e}")
            sys.exit(1)

    if mismatched_packages:
        print("The following packages do not match the versions specified in requirements.txt:")
        for name, required, installed in mismatched_packages:
            required_version = ", ".join([f"{op}{ver}" for op, ver in required])
            print(f"- {name}: required {required_version}, installed {installed}")
        sys.exit(1)
    else:
        print("All packages match the versions specified in requirements.txt.")
        print("Validation complete. Proceeding with application...")


# Example of calling this function from another module
if __name__ == "__main__":
    validate_requirements('../../requirements.txt')
