from setuptools import setup, find_packages

# Read README for long description
try:
    with open("README.md", "r", encoding="utf-8") as fh:
        long_description = fh.read()
except FileNotFoundError:
    long_description = "Binary serialization for FEAGI protocols"

setup(
    name="feagi_bytes",
    version="0.1.0",
    author="Neuraville Inc.",
    author_email="info@feagi.org",
    description="Binary serialization for FEAGI protocols",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/feagi/feagi",
    project_urls={
        "Bug Tracker": "https://github.com/feagi/feagi/issues",
    },
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: Apache Software License",
        "Operating System :: OS Independent",
    ],
    packages=find_packages(),
    python_requires=">=3.8",
    install_requires=[
        "numpy>=1.20.0",
    ],
) 