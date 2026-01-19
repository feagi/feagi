"""
Setup configuration for feagi-bv.

This marks the wheel as non-pure so it is tagged with the host platform,
ensuring platform-specific Brain Visualizer binaries are distributed.
"""

from wheel.bdist_wheel import bdist_wheel as _bdist_wheel
from setuptools import setup


class bdist_wheel(_bdist_wheel):
    """Force platform-specific wheel tags for binary payloads."""

    def finalize_options(self) -> None:
        super().finalize_options()
        self.root_is_pure = False


setup(
    cmdclass={"bdist_wheel": bdist_wheel},
    zip_safe=False,
)
