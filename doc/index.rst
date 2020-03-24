.. py:currentmodule:: lsst.ts.ATMCSSimulator

.. _lsst.ts.ATMCSSimulator:

######################
lsst.ts.ATMCSSimulator
######################

A simulator for the auxiliary telescope motor control system (ATMCS CSC).

.. _lsst.ts.ATMCSSimulator-using:

Using lsst.ts.ATMCSSimulator
============================

The package is compatible with LSST DM's ``scons`` build system and ``eups`` package management system.
Assuming you have the basic LSST DM stack installed you can do the following, from within the package directory:

* ``setup -r .`` to setup the package and dependencies.
* ``scons`` to build the package and run unit tests.
* ``scons install declare`` to install the package and declare it to eups.
* ``package-docs build`` to build the documentation.
  This requires ``documenteer``; see `building single package docs`_ for installation instructions.

.. _building single package docs: https://developer.lsst.io/stack/building-single-package-docs.html

With the package built and set up you can run the simulator using:

    run_atmcs_simulator.py

.. _lsst.ts.ATMCSSimulator-contributing:

Contributing
============

``lsst.ts.ATMCSSimulator`` is developed at https://github.com/lsst-ts/ts_ATMCSSimulator.
You can find Jira issues for this module using `labels=ts_ATMCSSimulator <https://jira.lsstcorp.org/issues/?jql=project%20%3D%20DM%20AND%20labels%20%20%3D%20ts_ATMCSSimulator>`_.

.. _lsst.ts.ATMCSSimulator-pyapi:

Python API reference
====================

.. automodapi:: lsst.ts.ATMCSSimulator
   :no-main-docstr:
   :no-inheritance-diagram:

.. automodapi:: lsst.ts.ATMCSSimulator.path
   :no-main-docstr:
   :no-inheritance-diagram:

Revision History
================

.. toctree::
    revision_history
    :maxdepth: 1
