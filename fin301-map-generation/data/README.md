# Included original recordings

Both complete source SQLite databases are stored here as gzip chunks, at most 512 MiB each.
`archives.json` records compressed-part hashes and original database hashes. Git LFS tracks
all `*.gz.part*` files through the repository `.gitattributes` file.

After `git clone` and `git lfs pull`, run `bash run_mapping.sh --dataset both` from the project
root. The runner automatically restores both databases to the paths below and verifies that
they are byte-identical to the originals:

- `fin301/fin301_0.db3`
- `Fin301-11051740/Fin301-11051740_0.db3`

For restoration only: `python3 src/restore_data.py`.

Restored `.db3` files are ignored by Git to avoid uploading duplicate copies. Commit the
compressed chunks and the JSON manifest instead. No other dataset is included. IMU is still
excluded by the mapping configuration even though the complete recordings preserve all topics.
