# # -*- coding: utf-8 -*-
# from pkg_resources import get_distribution, DistributionNotFound
#
# # Set a default value for __version__
# __version__ = 'unknown'
#
# # Try to obtain the installed package's version
# try:
#     # The package name is assumed to be the name of the directory the file is in
#     package_name = __name__  # If the package name differs, manually set this variable
#     __version__ = get_distribution(package_name).version
# except DistributionNotFound:
#     # If the package is not installed (e.g., running from source), the version will remain 'unknown'
#     pass
#
# # Clean-up: Remove imported modules to keep the namespace clean
# del get_distribution, DistributionNotFound
