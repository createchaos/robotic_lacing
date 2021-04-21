# Robotic lacing

We make lace with robots

## Installation
- Rhino 6 or later
- Compas: 1.0.0
- CompasFab: 0.16.0
- Robots plugin for grasshopper (temporary)
- CREATE Lab ABB communication library

1. Clone this repo
2. `cd` to the repo locally
3. `pip install -e .` (don't forget the dot at the end)
4. Install skspatial: `conda install -c anaconda scikit-image`
5. `python -m compas_rhino.install -p robotic_lacing`

## Not needed now, but later for visualization
- Docker (requires Windows Pro)
`cd {{ path to repository }}\docker\abb_120`
`docker-compose up -d`
