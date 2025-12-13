# Configuration file for the Sphinx documentation builder.

import os
import sys

# Add the source directory to path
sys.path.insert(0, os.path.abspath('.'))

project = 'Romi Autonomous Navigation'
copyright = '2024'
author = 'ME 405 Team'
release = '1.0.0'

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.viewcode',
    'sphinx.ext.mathjax',
]

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

html_theme = 'alabaster'
html_static_path = [] 
html_theme_options = {
    'github_user': 'shwanpla',
    'github_repo': 'ME-405-Term-Project',
}

master_doc = 'index'  