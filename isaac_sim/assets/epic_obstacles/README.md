# Epic obstacle assets

These assets were converted from the downloaded StarCraft 2 `scene.usdc`
files.

Use `obstacle.usd` when placing the object in an Isaac Sim terrain.  It wraps
the raw converted asset with:

- Z-up orientation for the rover world
- centimeters-to-meters scale
- ground-aligned origin
- an invisible box collider for stable static obstacle physics

`scene.usd` is the raw ASCII USD conversion and keeps the original material
texture references under `0/`.

| Asset | Placement USD | Approx. footprint | Height |
| --- | --- | ---: | ---: |
| Battlecruiser | `battlecruiser_starcraft2/obstacle.usd` | 5.18 m x 7.00 m | 3.62 m |
| Goliath | `goliath_blackops/obstacle.usd` | 5.69 m x 6.00 m | 6.74 m |
| SCV | `scv_starcraft2/obstacle.usd` | 5.00 m x 4.17 m | 4.13 m |
| Barracks | `barracks_starcraft2/obstacle.usd` | 7.00 m x 4.91 m | 4.61 m |
