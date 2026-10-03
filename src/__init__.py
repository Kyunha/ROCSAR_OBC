"""ROCSAR On-Board Computer (OBC) server package.

Runs on the Raspberry Pi 4B on board the gondola. Provides:

* a ZMQ ROUTER command endpoint (:mod:`src.obc_server`),
* a ZMQ PUB telemetry downlink,
* an HTTP file server for SSD data retrieval,
* a serial ASCII interface to the Raspberry Pi Pico control board
  (:mod:`src.serial_pico`),
* a GNSS ingest listener for both the JSON feed and the ``Read_uB``
  ``NavData`` binary feed (:mod:`src.gnss_listener`),
* dynamic bandwidth control via Linux ``tc`` (:mod:`src.bandwidth_manager`).

The package is meant to be launched as ``python3 -m src.obc_server`` from the
repository root, which is what ``scripts/run_server.sh`` does.
"""

__all__ = [
    "bandwidth_manager",
    "config",
    "gnss_listener",
    "serial_pico",
    "system_health",
]

__version__ = "0.1.0"
