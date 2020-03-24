.. py:currentmodule:: lsst.ts.ATMCSSimulator

.. _lsst.ts.ATMCSSimulator.revision_history:

##################################
ts_ATMCSSimulator Revision History
##################################

v0.11.0
=======

Major changes:

* Update for a change to the XML.
* Updated test_csc.py to use `lsst.ts.salobj.BaseCscTestCase`.
* Added a revision history.
* Code formatted by ``black``, with a pre-commit hook to enforce this. See the README file for configuration instructions.

Requires:

* ts_salobj 5.4
* ts_simactuators 0.1
* ts_idl 1
* ts_xml 4.9
* IDL file for ATMCS, e.g. built with make_idl_files.py


v0.10.0
=======

Major changes:

* Update to use ts_simactuators.
* Update unit tests to use asynctest.

Requires:

* ts_salobj 5.2
* ts_simactuators 0.1
* ts_idl 1
* IDL file for ATMCS, e.g. built with make_idl_files.py

v0.9.0
======

Major changes:

* Update for ts_salobj 5.2.
* Use simulation_mode instead of initial_simulation_mode

Requires:

* ts_salobj 5.2
* ts_idl 1
* IDL file for ATMCS, e.g. built with make_idl_files.py

v0.8.3
======

Make bin/run_atmcs_simulator.py executable (chmod +x).

Requirements:
* ts_salobj 4.5 or 5
* ts_idl
* IDL file for ATMCS, e.g. built with make_idl_files.py

v0.8.2
======

Major changes:

* Allow using the package without scons.

Other changes:

* Fix a bug in TPVAJ.pva.
* Modernize calling `BaseCsc.fault` to simplify the code and eliminate a deprecation warning.

Requirements:

* ts_salobj 4.5 or 5
* ts_idl
* IDL file for ATMCS, e.g. built with make_idl_files.py

v0.8.1
======

* Make sure M3 moves always display "in motion" state.
* Fix a unit test broken by a new generic event.

Requirements:

* ts_salobj v4.4
* ts_idl
* IDL file for ATMCS, e.g. built with make_idl_files.py

v0.8.0
======

Major changes:

Output the new positionLimits event.

Requirements:

* ts_salobj v4.4
* ts_idl
* IDL file for ATMCS, e.g. built with make_idl_files.py

v0.7.0
======

Major changes:

* Update for changes to ATMCS topics
* Most telemetry topic fields are now arrays.
* Added a few fields to the trackTarget command and target event.

Requirements:

* ts_salobj v4.4 or later
* ts_idl
* ATMCS IDL files, e.g. built with make_idl_files.py

v0.6.0
======

Major changes:

* Use OpenSplice dds.
* Do not enable unused axes.

Requirements:

* ts_salobj 4
* ts_idl
