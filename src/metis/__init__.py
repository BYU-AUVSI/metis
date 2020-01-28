# -*- coding: utf-8 -*-
# Copyright 2018-2019 John Akagi and Jacob Willis
# Copyright 2019-2020 Sequoia Ploeg

"""
Metis
"""

import logging

from metis.core import *

# Set up logger
logger = logging.getLogger('METIS')
logger.setLevel(logging.DEBUG)
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)
formatter = logging.Formatter('[%(levelname)s] [%(asctime)s] %(name)s: %(message)s')
ch.setFormatter(formatter)
logger.addHandler(ch)

logger.info("Metis initialized.")