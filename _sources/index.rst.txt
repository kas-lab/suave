Welcome to SUAVE's documentation!
=================================

SUAVE (Self-adaptive Underwater Autonomous Vehicle Exemplar) is a ROS 2 Humble
exemplar for self-adaptive robotics. An autonomous underwater vehicle searches
for and inspects a pipeline while thrusters fail, water visibility changes, and
its battery drains. SUAVE separates the managed vehicle functionality from
pluggable managing subsystems, so different adaptation approaches can be
compared through common ROS 2 interfaces.

Quick start
-----------

Read :doc:`mission` for what the vehicle does and which uncertainties it faces.
The Docker images are the fastest way to try SUAVE: see :doc:`docker`, then
:doc:`run` for missions and experiment campaigns. For a local installation,
follow :doc:`installation`.

.. toctree::
 :maxdepth: 1

 Home <self>
 mission
 installation
 docker
 cli
 run
 results
 metrics
 architecture
 extend
 implementations
 troubleshooting
 citing
 api

Contribute
----------

- Issue Tracker: https://github.com/kas-lab/suave/issues
- Source Code: https://github.com/kas-lab/suave/

Support
-------

If you are having issues, please let us know by creating an issue on GitHub.

Related work
------------

- `REMARO Summer School Delft 2022 <https://github.com/remaro-network/tudelft_hackathon>`_: underwater robotics hackathon

License
-------

The project is licensed under the Apache 2.0 license.

Acknowledgments
----------------
.. raw:: html

 <embed>
   <a href="https://remaro.eu/">
       <img height="60" alt="REMARO Logo" src="https://remaro.eu/wp-content/uploads/2020/09/remaro1-right-1024.png">
   </a>
 </embed>

This work is part of the Reliable AI for Marine Robotics (REMARO) Project. For more info, please visit: https://remaro.eu/

.. raw:: html

 <embed>
   <a href="https://research-and-innovation.ec.europa.eu/funding/funding-opportunities/funding-programmes-and-open-calls/horizon-2020_en">
       <img align="left" height="60" alt="EU Flag" src="https://remaro.eu/wp-content/uploads/2020/09/flag_yellow_low.jpg">
   </a>
 </embed>

This project has received funding from the European Union's Horizon 2020 research and innovation programme under the Marie Skłodowska-Curie grant agreement No. 956200.
