define laser ranger(
  sensor(
    range [ 0.0 2.0 ]
    fov 360.0
    samples 360
  )

  color "blue"
  size [ 0.12 0.12 0.06 ]

 block( points 8
    point[0] [0 0]
    point[1] [0 2]
    point[2] [1 3]
    point[3] [3 3]
    point[4] [4 2]
    point[5] [4 0]
    point[6] [3 -1]
    point[7] [1 -1]
    z [0 1]
  )
)

define simple_robot position(
  odom_error [0.03 0.03 0.00 0.05]
  localization "odom"
  size [ 0.4 0.4 0.24 ]
  gui_nose 1
  drive "diff"
  color "white"

  block( points 8
    point[0] [0 0]
    point[1] [0 2]
    point[2] [1 3]
    point[3] [3 3]
    point[4] [4 2]
    point[5] [4 0]
    point[6] [3 -1]
    point[7] [1 -1]
    z [0 1]
  )

    block( points 8
    point[0] [0 0]
    point[1] [0 2]
    point[2] [1 3]
    point[3] [3 3]
    point[4] [4 2]
    point[5] [4 0]
    point[6] [3 -1]
    point[7] [1 -1]
    z [0 2]
  )

  laser(pose [ 0.0 0.0 0.0 0.0 ])

)
