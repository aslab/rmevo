from __future__ import absolute_import

import logging
import sys


def create_logger(name='rmevo_core', level=logging.DEBUG, handlers=None):
    _logger = logging.getLogger(name)
    _logger.setLevel(level)
    handlers = logging.StreamHandler(sys.stdout) if handlers is None else handlers
    handlers = [handlers] if type(handlers) is not list else handlers
    for handler in handlers:
        _console_handler = handler
        _console_handler.setLevel(level)
        _revolve_formatter = logging.Formatter('[%(asctime)s %(name)10s] %(levelname)-8s %(message)s')
        _console_handler.setFormatter(_revolve_formatter)
        _logger.addHandler(_console_handler)
    return _logger


# General logger to standard output
logger = create_logger(
    name='rmevo_core',
    level=logging.DEBUG,
    handlers=[logging.StreamHandler(sys.stdout), logging.FileHandler('./rmevo_core.log', mode='a')]
)