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

import logging

# class EmojiLogger(logging.Logger):
#  def _log(self, level, msg, args, exc_info=None, extra=None,
#  stack_info=False, emoji1=None):
#         if extra is None:
#             extra = {}
#         extra['emoji'] = emoji or "  "
#         super()._log(level, msg, args, exc_info, extra, stack_info)

# class EmojiFormatter(logging.Formatter):
#     def format(self, record):
#         emoji = getattr(record, 'emoji', "  ")
#         timestamp = self.formatTime(record, self.datefmt)
#         return f"{emoji} [{timestamp}] {record.getMessage()}"


def setup_feagi_logging():
    # logging.setLoggerClass(EmojiLogger)
    handler = logging.StreamHandler()
    # handler.setFormatter(EmojiFormatter("%(message)s", "%Y-%m-%d %H:%M:%S"))
    root = logging.getLogger()
    root.handlers = [handler]
    root.setLevel(logging.WARNING)
