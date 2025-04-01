# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'META-SEJONG AI Robotics Challenge'
copyright = '2024, Sejong University'
author = 'Sejong University'
release = '1.0.0'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',
    'sphinx.ext.viewcode',
    'sphinx.ext.todo',
    'sphinx.ext.intersphinx',
    'sphinx.ext.mathjax',
    'sphinx.ext.githubpages',
    'myst_parser',
]

templates_path = ['_templates']
exclude_patterns = []

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']

# -- Options for LaTeX output ------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-latex-output

latex_elements = {
    'papersize': 'a4paper',
    'pointsize': '11pt',
    'figure_align': 'htbp',
    'preamble': r'''
        \usepackage{kotex}
        \usepackage{graphicx}
        \usepackage{amsmath}
        \usepackage{amssymb}
        \usepackage{booktabs}
        \usepackage{multirow}
        \usepackage{enumitem}
        \usepackage{listings}
        \usepackage{xcolor}
        \usepackage{hyperref}
        
        % 한글 폰트 설정
        \setmainfont{Noto Serif CJK KR}
        \setsansfont{Noto Sans CJK KR}
        
        % 코드 블록 스타일
        \lstset{
            basicstyle=\ttfamily\small,
            breaklines=true,
            breakatwhitespace=true,
            showstringspaces=false,
            frame=single,
            numbers=left,
            numberstyle=\tiny,
            keywordstyle=\color{blue},
            commentstyle=\color{green!60!black},
            stringstyle=\color{red},
            backgroundcolor=\color{gray!5},
        }
        
        % 페이지 레이아웃 설정
        \geometry{
            top=2.5cm,
            bottom=2.5cm,
            left=2.5cm,
            right=2.5cm,
            headheight=1.5cm,
            headsep=0.5cm,
            footskip=1cm,
            marginparwidth=2cm,
            marginparsep=0.5cm,
        }
        
        % 헤더/푸터 설정
        \pagestyle{fancy}
        \fancyhf{}
        \fancyhead[L]{\leftmark}
        \fancyhead[R]{\thepage}
        \fancyfoot[C]{\thepage}
        
        % 목차 설정
        \renewcommand{\contentsname}{Table of Contents}
        \renewcommand{\chaptername}{Chapter}
        \renewcommand{\sectionname}{Section}
        \renewcommand{\subsectionname}{Subsection}
        \renewcommand{\appendixname}{Appendix}
        
        % 그림/표 캡션 설정
        \renewcommand{\figurename}{Figure}
        \renewcommand{\tablename}{Table}
        
        % 수식 설정
        \renewcommand{\equationname}{Equation}
        \renewcommand{\eqref}[1]{Equation~\ref{#1}}
        
        % 참조 설정
        \renewcommand{\refname}{References}
        \renewcommand{\bibname}{Bibliography}
        
        % 하이퍼링크 설정
        \hypersetup{
            colorlinks=true,
            linkcolor=blue,
            filecolor=magenta,
            urlcolor=cyan,
            pdftitle={META-SEJONG AI Robotics Challenge Documentation},
            pdfauthor={Sejong University},
            pdfsubject={AI Robotics Challenge Documentation},
            pdfkeywords={AI, Robotics, Competition, Documentation},
        }
    ''',
}

# -- Options for autodoc extension -------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/extensions/autodoc.html#configuration

autodoc_default_options = {
    'members': True,
    'member-order': 'bysource',
    'special-members': '__init__',
    'undoc-members': True,
    'exclude-members': '__weakref__'
}

# -- Options for napoleon extension -----------------------------------------
# https://www.sphinx-doc.org/en/master/usage/extensions/napoleon.html#configuration

napoleon_google_docstring = True
napoleon_numpy_docstring = True
napoleon_include_init_with_doc = True
napoleon_include_private_with_doc = True
napoleon_include_special_with_doc = True
napoleon_use_admonition_for_examples = True
napoleon_use_admonition_for_notes = True
napoleon_use_admonition_for_references = True
napoleon_use_ivar = True
napoleon_use_param = True
napoleon_use_rtype = True
napoleon_type_aliases = None

# -- Options for intersphinx extension --------------------------------------
# https://www.sphinx-doc.org/en/master/usage/extensions/intersphinx.html#configuration

intersphinx_mapping = {
    'python': ('https://docs.python.org/3', None),
    'numpy': ('https://numpy.org/doc/stable/', None),
    'scipy': ('https://docs.scipy.org/doc/scipy/', None),
    'matplotlib': ('https://matplotlib.org/stable/', None),
    'pandas': ('https://pandas.pydata.org/docs/', None),
    'sklearn': ('https://scikit-learn.org/stable/', None),
    'torch': ('https://pytorch.org/docs/stable/', None),
    'tensorflow': ('https://www.tensorflow.org/guide', None),
    'opencv': ('https://docs.opencv.org/4.x/', None),
    'ros2': ('https://docs.ros.org/en/humble/', None),
}

# -- Options for todo extension ---------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/extensions/todo.html#configuration

todo_include_todos = True

# -- Options for mathjax extension ------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/extensions/mathjax.html#configuration

mathjax_path = 'https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js'
mathjax_options = {
    'processHtmlClass': 'math',
    'processEnvironments': True,
    'skipTags': ['script', 'noscript', 'style', 'textarea', 'pre', 'code'],
}

# -- Options for githubpages extension --------------------------------------
# https://www.sphinx-doc.org/en/master/usage/extensions/githubpages.html#configuration

githubpages_url = 'https://sejonguniversity.github.io/metasejong-competition.readthedocs.io/'

# -- Options for myst-parser extension --------------------------------------
# https://myst-parser.readthedocs.io/en/latest/configuration.html

myst_enable_extensions = [
    'colon_fence',
    'deflist',
    'dollarmath',
    'fieldlist',
    'html_admonition',
    'html_image',
    'replacements',
    'smartquotes',
    'substitution',
    'tasklist',
]

myst_url_schemes = ['http', 'https', 'mailto', 'ftp']
myst_heading_anchors = 3
myst_footnote_transition = True
myst_dmath_double_inline = True
myst_enable_checkboxes = True 