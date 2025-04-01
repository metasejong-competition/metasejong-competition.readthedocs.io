# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'META-SEJONG AI Robotics Challenge'
copyright = '2025, META-SEJONG AI Robotics Challenge'
author = 'META-SEJONG AI Robotics Challenge'

# The master toctree document
master_doc = 'index'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'myst_parser',
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',
    'sphinx.ext.viewcode',
    'sphinx.ext.githubpages',
]

templates_path = ['_templates']
exclude_patterns = []

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']

# HTML 테마 설정
html_theme_options = {
    'style_nav_header_background': '#2980B9',
    'navigation_depth': 4,
    'collapse_navigation': True,
    'sticky_navigation': True,
    'includehidden': True,
    'titles_only': False
}

# Enable copy button on code blocks
copybutton_prompt_text = r">>> |\.\.\. |\$\ |In \[\d*\]: | {2,5}\.\.\.: | {5,8}: "
copybutton_prompt_is_regexp = True

# Enable MyST Markdown features
myst_enable_extensions = [
    "colon_fence",
    "deflist",
    "dollarmath",
    "fieldlist",
    "html_admonition",
    "html_image",
    "replacements",
    "smartquotes",
    "substitution",
    "tasklist",
]

# LaTeX 설정
latex_elements = {
    'preamble': r'''
    \usepackage{kotex}
    \usepackage{setspace}
    \setstretch{1.2}
    \setlength{\headheight}{15pt}
    \setlength{\headsep}{0.5in}
    \setlength{\textheight}{8.5in}
    \setlength{\textwidth}{6.5in}
    \setlength{\topmargin}{0in}
    \setlength{\oddsidemargin}{0in}
    \setlength{\evensidemargin}{0in}
    ''',
    'papersize': 'a4paper',
    'pointsize': '11pt',
    'figure_align': 'htbp',
    'fontpkg': r'''
    \usepackage[T1]{fontenc}
    \usepackage{times}
    \usepackage[utf8]{inputenc}
    ''',
    'sphinxsetup': 'verbatimwithframe=false',
    'extraclassoptions': 'openany,oneside'
}

latex_documents = [
    (master_doc, 'metasejong-competition.tex', project,
     author, 'manual'),
]

latex_engine = 'pdflatex' 