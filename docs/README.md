# Using Sphinx to build Doc site

If you create a new python file, that file will need to be included in the
documentation by running this command

~~~~~~~~~~~~~~~~~~~~~~~~~bash
sphinx-apidoc -o docs/ . # from project root
# or
sphinx-apidoc -o . ..    # from docs dir
~~~~~~~~~~~~~~~~~~~~~~~~~

Following this or if you only modified a python file instead of created a new
one, then you can locally build the doc site using the following command

~~~~~~~~~~~~~~~~~~~bash
make -C docs/ html # from project root
# or
make html          # from docs dir
~~~~~~~~~~~~~~~~~~~

this will build the Doc site in `docs/_build/html`.
