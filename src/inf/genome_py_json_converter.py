#!/usr/bin/python
"""

Converts a genome saved in Python format to JSON

usage:

"""
import sys
import json
from importlib import util

try:
    file_path = sys.argv[1]
    file_name = sys.argv[2]

    full_path = file_path + file_name


    def set_default(obj):
        if isinstance(obj, set):
            return list(obj)
        raise TypeError


    spec = util.spec_from_file_location("module.name", full_path)
    foo = util.module_from_spec(spec)
    sys.modules["module.name"] = foo
    spec.loader.exec_module(foo)

    filtered_genome = foo.genome

    dst_file_name = file_name[:-3] + ".json"

    try:
        with open(file_path + dst_file_name, "w") as data_file:
            data_file.seek(0)  # rewind
            data_file.write(json.dumps(filtered_genome, indent=3, default=set_default))
            data_file.truncate()

            print("genome is saved as ", dst_file_name)

    except KeyError:
        print("Warning: Genome could not be saved!")

except IndexError:
    print("Incorrect usage!")
    print("This scripts takes two arguments with first being the file path and second the file name")
