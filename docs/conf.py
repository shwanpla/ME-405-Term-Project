# Configuration file for the Sphinx documentation builder.

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
html_static_path = ['_static']
html_theme_options = {
    'github_user': 'shwanpla',
    'github_repo': 'ME-405-Term-Project',
}

master_doc = 'index'
