import logging


class LevelFilter(logging.Filter):
    def __init__(self, level):
        super().__init__()
        self.level = level

    def filter(self, record):
        return record.levelno == self.level


def setup_logger():
    logger = logging.getLogger("AEB")
    logger.setLevel(logging.DEBUG)
    logger.propagate = False

    if not logger.handlers:
        formatter = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")

        critical_handler = logging.FileHandler(
            "logs/aeb_critical.log", encoding="utf-8"
        )
        critical_handler.setLevel(logging.CRITICAL)
        critical_handler.setFormatter(formatter)
        critical_handler.addFilter(LevelFilter(logging.CRITICAL))

        warning_handler = logging.FileHandler("logs/aeb_warning.log", encoding="utf-8")
        warning_handler.setLevel(logging.WARNING)
        warning_handler.setFormatter(formatter)
        warning_handler.addFilter(LevelFilter(logging.WARNING))

        logger.addHandler(critical_handler)
        logger.addHandler(warning_handler)

    return logger


logger = setup_logger()
