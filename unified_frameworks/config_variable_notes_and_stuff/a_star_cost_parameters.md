# Parameters being considered in the cost calculations

- Arrival cost to a point
- Heuristic cost from a point
- Collision potential of a point
  - defined as 1/(nearest distance obstacle)^n
- Turning cost
- Step size in meters
- Neighbor sector, the angle by which to deviate for the next step
- Number of neighbors

## A really good configuration

- 1 * Arrival cost
- 4 * Heuristic cost
- 0.3 * Collision potential (1/(d*0.2))
- 0.2m step size
- +-90 degree neighbor sector
- 6 neighbors

* 0.5m on on the Lidar is near enough for a collision. Account for that

* raising the heuristic to a power is a pretty effective way motivating the algorithm in a particular direction and not explore around the same spot


## Things to try out

- path smoothing/fixing algorithms.
  - Something like, once you have a path, modify the path to avoid obstacles better