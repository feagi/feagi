
#
# Copyright 2016-Present Neuraville Inc. All Rights Reserved.
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


import gzip
import shutil
import argparse
import os
import json


def unzip_and_pretty_print_gzip(input_file):
    output_file = os.path.splitext(input_file)[0] + ".json"  # Remove .gz extension

    # Unzip the gzip file
    with gzip.open(input_file, 'rb') as f_in:
        with open(output_file, 'wb') as f_out:
            shutil.copyfileobj(f_in, f_out)

    # Read the JSON content from the unzipped file
    with open(output_file, 'r') as f:
        data = json.load(f)

    # Pretty print the JSON content back to the unzipped file
    with open(output_file, 'w') as f:
        json.dump(data, f, indent=4)

    print(f"File '{input_file}' has been unzipped and pretty-printed to '{output_file}'")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Unzip a gzip file and pretty-print its JSON content.")
    parser.add_argument("input_file", help="The path to the gzip file to be unzipped.")

    args = parser.parse_args()

    unzip_and_pretty_print_gzip(args.input_file)
