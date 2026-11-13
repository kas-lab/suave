# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

import datetime
import pathlib
import re

project = 'SUAVE'
author = 'Gustavo Rezende, Juliane Päßler, Jeroen Zwanepol, Elvin Alberts, Lizeth Tarifa, Ilias Gerostathopoulos, Einar Johnsen, Carlos Hernández'
copyright = f'2023-{datetime.date.today().year}, {author}'

# The release is the most recent version heading in the CHANGELOG.
_changelog = pathlib.Path(__file__).resolve().parents[2] / 'CHANGELOG.md'
_version = re.search(r'^## (\d+\.\d+\.\d+)', _changelog.read_text(encoding='utf-8'), re.MULTILINE)
release = _version.group(1)

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    "sphinx.ext.duration",
    "sphinx.ext.doctest",
    "sphinx.ext.autodoc",
    "sphinx.ext.autosummary",
    "myst_parser",
]
autosummary_generate = True
autodoc_mock_imports = ["pandas", "scipy"]

templates_path = ["_templates"]

# -- Options for EPUB output
epub_show_urls = "footnote"

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]

# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = "sphinx_rtd_theme"

source_suffix = {
    '.rst': 'restructuredtext',
    '.txt': 'markdown',
    '.md': 'markdown',
}
myst_heading_anchors = 3
