.. py:currentmodule:: lsst.ts.atmcs

.. _lsst.ts.atmcs:

#############
lsst.ts.atmcs
#############

.. image:: https://img.shields.io/badge/Project Metadata-gray.svg
    :target: https://ts-xml.lsst.io/index.html#index-master-csc-table-atmcs
.. image:: https://img.shields.io/badge/SAL\ Interface-gray.svg
    :target: https://ts-xml.lsst.io/sal_interfaces/ATMCS.html
.. image:: https://img.shields.io/badge/GitHub-gray.svg
    :target: https://github.com/lsst-ts/ts_atmcs
.. image:: https://img.shields.io/badge/Jira-gray.svg
    :target: https://jira.lsstcorp.org/issues/?jql=project%3DDM%20AND%20labels%3Dts_atmcs

A CSC for the auxiliary telescope motor control system (ATMCS).

.. _lsst.ts.atmcs-using:

User Guide
==========

The package is compatible with LSST DM's ``scons`` build system and ``eups`` package management system.
Assuming you have the basic LSST DM stack installed you can do the following, from within the package directory:

* ``setup -r .`` to setup the package and dependencies.
* ``scons`` to build the package and run unit tests.
* ``scons install declare`` to install the package and declare it to eups.
* ``package-docs build`` to build the documentation.
  This requires ``documenteer``; see `building single package docs`_ for installation instructions.

.. _building single package docs: https://developer.lsst.io/stack/building-single-package-docs.html

With the package built and set up you can run the CSC using:

    run_atmcs

If you want a different configuration, you can call `ATMCSCsc.configure` (from Python, not SAL).

.. _lsst.ts.atmcs-contributing:

Contributing
============

``lsst.ts.atmcs`` is developed at https://github.com/lsst-ts/ts_atmcs.
You can find Jira issues for this module using `project=DM and labels=ts_atmcs <https://jira.lsstcorp.org/issues/?jql=project%3DDM%20AND%20labels%3Dts_atmcs>`_.

.. _lsst.ts.atmcs-pyapi:

Python API reference
====================

.. automodapi:: lsst.ts.atmcs
   :no-main-docstr:

Version History
===============

.. toctree::
    version_history
    :maxdepth: 1
