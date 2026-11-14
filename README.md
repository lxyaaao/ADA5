# ADA5 — xinyu

This branch adds the **fin301 camera + 2D LiDAR mapping and temporal-filtering project** in [fin301-temporal-mapping/](fin301-temporal-mapping/README.md).

- [English offline viewer](fin301-temporal-mapping/OPEN_VIEWER.html): download or clone the repository, then open this file locally. GitHub's source preview does not run the viewer.
- [Conservative PCD](fin301-temporal-mapping/outputs/fin301/temporal_filter/conservative/result/map.pcd), [LiDAR-only PCD](fin301-temporal-mapping/outputs/fin301/temporal_filter/conservative/result/lidar_scan_map.pcd), [YAML](fin301-temporal-mapping/outputs/fin301/temporal_filter/conservative/result/map.yaml), [PGM](fin301-temporal-mapping/outputs/fin301/temporal_filter/conservative/result/map.pgm).
- [Temporal filtering method](fin301-temporal-mapping/outputs/fin301/temporal_filter/temporal_filter_report.md) and [localization comparison](fin301-temporal-mapping/outputs/fin301/temporal_filter/validation/temporal_localization_review.md).

The inherited [fin301-map-generation/](fin301-map-generation/) project and its Git LFS references are unchanged. The implementations and results have separate directories. The temporal project uses the original **fin301** recording; it does not claim results for `Fin301-11051740`.

To inspect the maps without downloading inherited multi-gigabyte LFS recordings:

```bash
GIT_LFS_SKIP_SMUDGE=1 git clone --branch xinyu https://github.com/lxyaaao/ADA5.git
```

PowerShell:

```powershell
$env:GIT_LFS_SKIP_SMUDGE = '1'
git clone --branch xinyu https://github.com/lxyaaao/ADA5.git
```

All new map and viewer files are ordinary Git files. Raw recordings are needed only for reconstruction, not for viewing the delivered maps.
