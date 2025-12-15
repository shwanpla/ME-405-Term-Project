# Configuration file for the Sphinx documentation builder.

import os
import sys

# --- Path setup --------------------------------------------------------------
# Point Sphinx to your source code directory (MicroPython modules live here)
sys.path.insert(0, os.path.abspath('../src'))

# --- Project information -----------------------------------------------------
project = 'ME 405 Term Project - Romi Navigation'
author = 'Billy Hawkins, Tarsem Pal, and Arturo Ramirez'
copyright = '2025'
release = '1.0.0'

# --- General configuration ---------------------------------------------------
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.viewcode',
    'sphinx.ext.mathjax',
]

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

# MicroPython modules do not exist on CPython,
# so we mock them so autodoc can import your files.
autodoc_mock_imports = [
    'pyb',
    'machine',
    'micropython',
    'utime',
    'uos',
    'ujson',
    'time',
    'ulab',
    'ulab.numpy',
    'cotask',
    'task_share',
    'encoder',
    'motor',
    'multi_sensor_read',
    'navigation',
    'observer_fcn',
    'serial_task_V6',
    'CL_control_task_V5',
    'motor_ctrl_task_V3',
]

# The master file (index)
master_doc = 'index'

# --- HTML output -------------------------------------------------------------
html_theme = 'sphinx_rtd_theme'
html_static_path = []  # You can add custom CSS later if desired

html_theme_options = {
    'github_user': 'shwanpla',
    'github_repo': 'ME-405-Term-Project',
}
