.. py:currentmodule:: lsst.ts.atmcs

.. _lsst.ts.atmcs.version_history:

###############
Version History
###############

.. towncrier release notes start

v2.1.2 (2025-10-09)
===================

Performance Enhancement
-----------------------

- Updated ts-conda-build dependency version and conda build string. (`OSW-1207 <https://rubinobs.atlassian.net//browse/OSW-1207>`_)


v2.1.1 (2025-09-30)
===================

Performance Enhancement
-----------------------

- Made NASMYTH2 the default port. (`OSW-1134 <https://rubinobs.atlassian.net//browse/OSW-1134>`_)


v2.1.0 (2025-09-10)
===================

Performance Enhancement
-----------------------

- Added support for changes in ts_attcpip. (`OSW-884 <https://rubinobs.atlassian.net//browse/OSW-884>`_)


v2.0.0 (2025-08-09)
===================

Performance Enhancement
-----------------------

- Complete overhaul of the code to implement a CSC that communicates with the cRIO via TCP/IP.

  * Modernize Jenkinsfile.
  * Add JSON schemas for command, event and telemetry exchange via TCP/IP.
  * Add MCS simulator and simulator server for testing the TCP/IP interaction with the real ATMCS server.
  * Move simulation code mostly unchanged from the CSC to the simulator.
  * Remove all simulator code from the CSC and connect to the simulator via TCP/IP.
  * Make ATMCS CSC a Configurable CSC.
  * Add support for sending start, disable, enable and standby commands.
  * Add support for summary state, detailed state and errorCode events and for FAULT state.
  * Support simulation mode 0.
  * Update the version of ts-conda-build to 0.4 in the conda recipe. (`DM-46935 <https://rubinobs.atlassian.net//browse/DM-46935>`_)


v1.5.2
------

* pre-commit: update black to 23.1.0 and pre-commit-hooks to v4.4.0 and add isort.
* ``Jenkinsfile``: stop running as root.

Requires:

* ts_salobj 7.1
* ts_utils 1
* ts_simactuators 2
* ts_idl 2
* IDL file for ATMCS built from ts_xml 11

v1.5.1
------

* Update ``ATMCSCsc.update_events`` to map the exit port enumeration name to the m3 state enumeration value when m3 is in position.
* Minor patch in ``ATMCSCsc.m3_port_rot`` to ignore ``m3_state`` when unpacking ``_port_info_dict``.

Requires:

* ts_salobj 7.1
* ts_utils 1
* ts_simactuators 2
* ts_idl 2
* IDL file for ATMCS built from ts_xml 11

v1.5.0
------

* Rename the package from ts_ATMCSSimulator to ts_atmcssimulator.
* Change reported cscVersion suffix from " sim" to "-sim".
* conda/meta.yaml: update to support multiple versions of Python.
* Jenkinsfile.conda: remove mention of ts_config_attcs; this CSC is not configurable.

Requires:

* ts_salobj 7.1
* ts_utils 1
* ts_simactuators 2
* ts_idl 2
* IDL file for ATMCS built from ts_xml 11

v1.4.0
------

* Rename command-line scripts to remove ".py" suffix.
* Build with pyproject.toml.

v1.3.1
------

* `ATMCSCsc`: call ``super().start()`` at the beginning of the start method.
  This requires ts_salobj 7.1.
* ``setup.cfg``: set asyncio_mode = auto.
* git ignore .hypothesis.
* Modernize ``Jenkinsfile``.

Requires:

* ts_salobj 7.1
* ts_utils 1
* ts_simactuators 2
* ts_idl 2
* IDL file for ATMCS built from ts_xml 11

v1.2.1
------

* tests/test_csc.py test_initial_state: update the list of initial events to skip to remove obsolete events.

Requires:

* ts_salobj 7
* ts_utils 1
* ts_simactuators 2
* ts_idl 2
* IDL file for ATMCS built from ts_xml 11

v1.2.0
------

* Update for ts_salobj v7, which is required.
  This also requires ts_xml 11.
* Use ts_utils and pytest-black.
* Modernize unit tests to use bare assert.

Requires:

* ts_salobj 7
* ts_utils 1
* ts_simactuators 2
* ts_idl 2
* IDL file for ATMCS built from ts_xml 11

v1.1.4
------

* Update `test_initial_info` in `test_csc` to ignore `largeFileObjectAvailable` new generic event (ts_xml >10).
* Update calls to methods that moved from `lsst.ts.salobj` to `lsst.ts.utils`.
* Modernize ``doc/conf.py`` for documenteer 0.6.

Requires:

* ts_salobj >=6
* ts_utils >=1
* ts_simactuators 2
* ts_idl >=2
* ts_xml >=5
* IDL file for ATMCS, e.g. built with make_idl_files.py

v1.1.3
------

* Use `unittest.IsolatedAsyncioTestCase` instead of the abandoned asynctest package.
* Use pre-commit instead of a custom pre-commit hook; see the README.md for instructions.
* Format the code with black 20.8b1.

Requires:

* ts_salobj 6
* ts_simactuators 2
* ts_idl 2
* ts_xml 5 - 6
* IDL file for ATMCS, e.g. built with make_idl_files.py
* Modernize ``doc/conf.py`` for documenteer 0.6.

v1.1.2
------

* `ATMCSCsc`: set class variable ``version`` to the package version + " sim", to differentiate between this and the real ATMCS CSC.
  Test that this properly sets the ``cscVersion`` field of the ``softwareVersions`` event.
* Make the initial position easily configurable.
* Make the initial elevation more realistic (the other actuators were fine).

Requires:

* ts_salobj 6
* ts_simactuators 2
* ts_idl 2
* ts_xml 5 - 6
* IDL file for ATMCS, e.g. built with make_idl_files.py
* Modernize ``doc/conf.py`` for documenteer 0.6.

v1.1.1
------

* Updated Jenkinsfile.conda to Jenkins Shared Library
* Pinned the version of ts-idl and ts-salobj in conda recipe

Requires:

* ts_salobj 6
* ts_simactuators 2
* ts_idl 2
* ts_xml 5 - 6
* IDL file for ATMCS, e.g. built with make_idl_files.py

v1.1.0
------

* Updated for ts_salobj 6.1.
* Updated `ATMCSCsc.set_event` to return ``did_put``, for debugging.
* Defined `ATMCSCsc` class variable ``valid_simulation_modes`` to eliminate a deprecation warning.
* Remove deprecation warnings caused by calling `salobj.RemoteTopic.get` with ``flush`` specified.
* Removed obsolete travis file.

Requires:

* ts_salobj 6
* ts_simactuators 2
* ts_idl 2
* ts_xml 5 - 6
* IDL file for ATMCS, e.g. built with make_idl_files.py

v1.0.4
------

* Update deprecated code for compatibility with ts_salobj 6 (and 5).
* Add black to conda test dependencies

Requires:

* ts_salobj 5.11 or 6.0
* ts_simactuators 1 or 2
* ts_idl 1 (for ts_salobj 5) or 2 (for ts_salobj 6)
* ts_xml 5 - 6
* IDL file for ATMCS, e.g. built with make_idl_files.py

v1.0.3
------

* Update for compatibility with ts_salobj 5.13.

Requires:

* ts_salobj 5.11
* ts_simactuators 1.0
* ts_idl 1
* ts_xml 5
* IDL file for ATMCS, e.g. built with make_idl_files.py

v1.0.2
------

* Add a test that code is formatted with black.
  This requires ts_salobj 5.11.
* Add a test for ``bin/run_atmcs_simulator.py``.
* Fix f strings with no {}.
* Remove ``sudo: false`` from ``.travis.yml``.

Requires:

* ts_salobj 5.11
* ts_simactuators 1.0
* ts_idl 1
* ts_xml 5
* IDL file for ATMCS, e.g. built with make_idl_files.py

v1.0.1
------

* Include conda package build configuration.
* Added a Jenkinsfile to support continuous integration and to build conda packages.
* Fix Jenkinsfile for CI job.

Requires:

* ts_salobj 5.4
* ts_simactuators 1.0
* ts_idl 1
* ts_xml 5
* IDL file for ATMCS, e.g. built with make_idl_files.py

v1.0.0
------=

First release. No changes from v0.11.0.

Requires:

* ts_salobj 5.4
* ts_simactuators 1.0
* ts_idl 1
* ts_xml 5
* IDL file for ATMCS, e.g. built with make_idl_files.py

v0.11.0
------=

Major * Update for a change to the XML.
* Updated test_csc.py to use `lsst.ts.salobj.BaseCscTestCase`.
* Added a revision history.
* Code formatted by ``black``, with a pre-commit hook to enforce this. See the README file for configuration instructions.

Requires:

* ts_salobj 5.4
* ts_simactuators 0.1
* ts_idl 1
* ts_xml 5
* IDL file for ATMCS, e.g. built with make_idl_files.py

v0.10.1
------=

Major * Added jenkins build.

Requires:

* ts_salobj 5.2
* ts_simactuators 0.1
* ts_idl 1
* IDL file for ATMCS, e.g. built with make_idl_files.py

v0.10.0
------=

Major * Update to use ts_simactuators.
* Update unit tests to use asynctest.

Requires:

* ts_salobj 5.2
* ts_simactuators 0.1
* ts_idl 1
* IDL file for ATMCS, e.g. built with make_idl_files.py

v0.9.0
------

Major * Update for ts_salobj 5.2.
* Use simulation_mode instead of initial_simulation_mode

Requires:

* ts_salobj 5.2
* ts_idl 1
* IDL file for ATMCS, e.g. built with make_idl_files.py

v0.8.3
------

Make bin/run_atmcs_simulator.py executable (chmod +x).

Requirements:
* ts_salobj 4.5 or 5
* ts_idl
* IDL file for ATMCS, e.g. built with make_idl_files.py

v0.8.2
------

Major * Allow using the package without scons.

Other * Fix a bug in TPVAJ.pva.
* Modernize calling `BaseCsc.fault` to simplify the code and eliminate a deprecation warning.

Requirements:

* ts_salobj 4.5 or 5
* ts_idl
* IDL file for ATMCS, e.g. built with make_idl_files.py

v0.8.1
------

* Make sure M3 moves always display "in motion" state.
* Fix a unit test broken by a new generic event.

Requirements:

* ts_salobj v4.4
* ts_idl
* IDL file for ATMCS, e.g. built with make_idl_files.py

v0.8.0
------

Major Output the new positionLimits event.

Requirements:

* ts_salobj v4.4
* ts_idl
* IDL file for ATMCS, e.g. built with make_idl_files.py

v0.7.0
------

Major * Update for changes to ATMCS topics
* Most telemetry topic fields are now arrays.
* Added a few fields to the trackTarget command and target event.

Requirements:

* ts_salobj v4.4 or later
* ts_idl
* ATMCS IDL files, e.g. built with make_idl_files.py

v0.6.0
------

Major * Use OpenSplice dds.
* Do not enable unused axes.

Requirements:

* ts_salobj 4
* ts_idl
