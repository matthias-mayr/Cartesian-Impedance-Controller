## Troubleshooting

### Multiple Gravity Definitions exist
```
gazebo: Warning [parser_urdf.cc:1115] multiple inconsistent <gravity> exists
due to fixed joint reduction overwriting previous value [true] with [false].
```

From what I understand this happens when the iiwa model is spawned into the world which creates some `<gravity>0</gravity>` entries.
[This issue seems to be related](https://bitbucket.org/osrf/gazebo/issues/1185/gzsdf-cant-disable-gravity-for-a-single). So for us it should be ignored and a `wontfix`.

