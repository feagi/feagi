#!/usr/bin/env python3
"""Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use
this file except in compliance with the License. You may obtain a copy of the
License at
http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""
FEAGI package entry point.

This module allows FEAGI to be run as:
    python -m feagi

It imports and executes the main function from main.py.
"""

import sys

if __name__ == "__main__":
    from feagi.main import main

    sys.exit(main())
