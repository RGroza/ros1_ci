## Passing Conditions

The passing conditions for position and rotation tests require a larger tolerance to be set:

*turtlebot_waypoints/src/test/end_position_test.py*

Line 14:
```
    self.pos_tolerance = 0.15
```

---

*turtlebot_waypoints/src/test/end_rotation_test.py*

Line 15-16:
```
    self.yaw_tolerance = math.radians(25)
    self.pos_tolerance = 0.15
```

---

## Failing Conditions

For the failing conditions, change the tolerances in the python testing scripts to lower values:

*turtlebot_waypoints/src/test/end_position_test.py*

Line 14:
```
    self.pos_tolerance = 0.01
```

---

*turtlebot_waypoints/src/test/end_rotation_test.py*

Line 15-16:
```
    self.yaw_tolerance = math.radians(1)
    self.pos_tolerance = 0.01
```