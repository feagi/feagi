#!/usr/bin/env bash
cd docs
sphinx-apidoc -o ./source/ ..
make clean
make html
cd ..
open ./docs/build/html/index.html
