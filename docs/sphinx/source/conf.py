#!/usr/bin/env python3
# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
import os
import sys
# sys.path.insert(0, os.path.abspath('.'))
sys.path.append(os.path.abspath('../../../src/factory_ros'))
sys.path.append(os.path.abspath('../../../src/rmevo'))
sys.path.append(os.path.abspath('../../../src/rmevo_gazebo'))
sys.path.append(os.path.abspath('../../sphinx_ros'))

import recommonmark
from recommonmark.transform import AutoStructify

# -- Project information -----------------------------------------------------

project = 'RMEvo Framework'
copyright = '2020, J. Poveda'
author = 'J. Poveda'

# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = ['sphinx.ext.autodoc',
            'sphinx.ext.autosummary',
            'sphinx.ext.todo',
            'sphinx_ros',
            'm2r2',
            'breathe']

import subprocess
subprocess.call('cd ../../doxygen ; doxygen', shell=True)

breathe_projects = { "RMEvoFramework": "../../doxygen/build/xml" }
breathe_default_project = "RMEvoFramework"

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# The suffix(es) of source filenames.
# You can specify multiple suffix as a list of string:
#
# source_suffix = ['.rst', '.md']
source_suffix = ['.rst', '.md']

# The master toctree document.
master_doc = 'index'

autosummary_generate = True

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = []

# The name of the Pygments (syntax highlighting) style to use.
pygments_style = 'sphinx'

todo_include_todos = True

autodoc_mock_imports = ["rospy",
                        "std_srvs",
                        "gazebo_msgs",
                        "rmevo",
                        "factory_ros"]

# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = 'sphinx_rtd_theme'
html_theme_options = {
    'navigation_depth' : 6
}

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['static']

# def setup(app):
#     app.add_config_value('recommonmark_config', {
#             'url_resolver': lambda url: github_doc_root + url,
#             'auto_toc_tree_section': 'Contents',
#             }, True)
#     app.add_transform(AutoStructify)

# -- Options for Latex output -------------------------------------------------

# Copy images
from shutil import copytree, copy2

def recursive_copy(src, dst, symlinks=False, ignore=None):
    for item in os.listdir(src):
        s = os.path.join(src, item)
        d = os.path.join(dst, item)
        if os.path.isdir(s):
            copytree(s, d, symlinks, ignore)
        else:
            try:
                copy2(s, d)
            except IOError as io_err:
                os.makedirs(os.path.dirname(d))
                copy2(s, d)

read_the_docs_build = os.environ.get('READTHEDOCS', None) == 'True'

if read_the_docs_build:
    print("Copying images")
    recursive_copy(os.path.abspath('../../../images'), os.path.abspath('../_build/images/'))
else:
    recursive_copy(os.path.abspath('../../../images'), os.path.abspath('../build/images/'))