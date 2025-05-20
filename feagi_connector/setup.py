from setuptools import setup, find_packages

# Read README for long description
try:
    with open("README.md", "r", encoding="utf-8") as fh:
        long_description = fh.read()
except FileNotFoundError:
    long_description = "Client library for connecting agents to FEAGI"

setup(
    name="feagi_connector",
    version="0.1.0",
    author="Neuraville Inc.",
    author_email="info@feagi.org",
    description="Client library for connecting agents to FEAGI",
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
        "feagi_bytes>=0.1.0",
        "numpy>=1.20.0",
        "pyzmq>=24.0.0",
    ],
) 