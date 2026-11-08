"""Build a fresh raw-scan-only bag for Cartographer; never modify source data."""
import sqlite3
import rosbag2_py
from context import BAG, WORK

destination = WORK / 'raw_scans'
if destination.exists():
    raise RuntimeError('Raw replay bag already exists; the pipeline manages cache invalidation.')
with sqlite3.connect('file:' + str(BAG) + '?mode=ro', uri=True) as source:
    row = source.execute("select id from topics where name='/scan/raw'").fetchone()
    if row is None:
        raise RuntimeError('Required /scan/raw topic is absent')
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(destination), storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    writer.create_topic(rosbag2_py.TopicMetadata(name='/scan', type='sensor_msgs/msg/LaserScan', serialization_format='cdr'))
    count = 0
    for timestamp, blob in source.execute('select timestamp,data from messages where topic_id=? order by timestamp', row):
        writer.write('/scan', blob, timestamp)
        count += 1
    del writer
print('Copied raw scans:', count, flush=True)
