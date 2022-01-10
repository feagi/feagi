# -*- coding: utf-8 -*-

import pytest
from src.skeleton import fib

__author__ = "Mohammad Nadji-Tehrani"
__copyright__ = "Mohammad Nadji-Tehrani"
__license__ = "apache"


def test_fib():
    # modifying this file to test CI linting filter
    assert fib(1) == 1
    assert fib(2) == 1
    assert fib(7) == 13
    with pytest.raises(AssertionError):
        fib(-10)
