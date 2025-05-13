import logging

# class EmojiLogger(logging.Logger):
#     def _log(self, level, msg, args, exc_info=None, extra=None, stack_info=False, emoji1=None):
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
    root.setLevel(logging.INFO) 