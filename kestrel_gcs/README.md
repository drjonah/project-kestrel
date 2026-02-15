# Project Kestrel: Ground Control System

## Project Structure

```
kestrel_gcs/
│
├── config/                 # Configuration files (avoid hardcoding)
│   ├── __init__.py
│   ├── settings.py         # Pin maps, Radio Addresses, frequencies
│   └── controller_map.py   # PS4 button mappings (Axis IDs, etc.)
│
├── data/                   # Where your logs live
│   └── telemetry_logs/     # Timestamped CSVs or SQLite dbs
│
├── src/                    # The actual source code
│   ├── __init__.py
│   │
│   ├── inputs/             # Hardware Abstraction Layer for User Input
│   │   ├── __init__.py
│   │   └── ps4_driver.py   # Wraps 'evdev' or 'pyPS4Controller'
│   │
│   ├── comms/              # Hardware Abstraction Layer for Radio
│   │   ├── __init__.py
│   │   ├── nrf24_driver.py # Low-level SPI interaction
│   │   └── radio_protocol.py # Packet encoding/decoding (struct packing)
│   │
│   ├── logging/            # Data persistence
│   │   ├── __init__.py
│   │   └── data_logger.py  # Handles writing to disk asynchronously
│   │
│   └── core/               # Business Logic
│       ├── __init__.py
│       └── state_manager.py # The 'Brain': holds current drone state vs target state
│
├── analysis/               # Offline scripts (run these after flying)
│   ├── plot_flight.py      # Matplotlib/Pandas scripts to visualize logs
│   └── replay_log.py       # Replay a flight from a log file
│
├── main.py                 # Entry point (orchestrator)
├── requirements.txt        # Python dependencies
└── README.md
```

## Quick Start

1. Create .venv

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install requirements.txt
```

2. Run

```bash
python3 main.py
```
