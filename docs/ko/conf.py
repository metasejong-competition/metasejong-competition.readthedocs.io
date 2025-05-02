import os
import sys
sys.path.insert(0, os.path.abspath('..'))

project = 'MARC (Meta-Sejong AI Robotics Challenge) 2025'
copyright = '2024, IOTCOSS(IoT Convergence & Open Sharing System)'
author = 'IOTCOSS'
release = '1.0.0'

# 한글 문서 설정
language = 'ko'
source_suffix = {
    '.md': 'markdown',
}

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',
    'sphinx.ext.viewcode',
    'sphinx.ext.intersphinx',
    'sphinx.ext.todo',
    'myst_parser',
    'sphinx_copybutton',
]

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']

intersphinx_mapping = {
    'python': ('https://docs.python.org/3/', None),
    'numpy': ('https://numpy.org/doc/stable/', None),
    'scipy': ('https://docs.scipy.org/doc/scipy/', None),
    'matplotlib': ('https://matplotlib.org/stable/', None),
    'pandas': ('https://pandas.pydata.org/docs/', None),
    'sklearn': ('https://scikit-learn.org/stable/', None),
    'torch': ('https://pytorch.org/docs/stable/', None),
    'ros2': ('https://docs.ros.org/en/humble/', None),
}

# MyST Parser 설정
myst_enable_extensions = [
    'colon_fence',
    'deflist',
    'fieldlist',
    'html_image',
    'substitution',
    'tasklist',
    'smartquotes',
    'replacements',
    'html_admonition',
]
myst_dmath_double_inline = True
myst_enable_checkboxes = True 