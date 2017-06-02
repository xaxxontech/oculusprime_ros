^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package oculusprime
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.1 (2017-06-01)
------------------
* readme modified
* floorplane_scan_enable mapping launch param change bugfix
* default astra horiz offset -1 deg
* horiz_angle_offset launch param setup (incomplete)
* recovery tweak
* remote_nav recovery modification
* follower parameter tweak, descriptions
* depth cam use_device_time default to false
* adapted turtlebot blob follower
* cmd_vel_listener reverse arcmove enabled
* cmd_vel_listener reverse arcmove enabled
* odom_test rviz profile really added this time
* odom_test rviz profile
* cmd_vel_listener node added
* cmd_vel_listener node added
* arcmove tweaks, false transient obstacle move fix
* arcmove tweaks, false transient obstacle move fix
* oculusprimesocket module reconnect fix
* oculusprimesocket module reconnect fix
* orbbec testing
* comment cleanup
* fix check for state rosarcmove
* transient obstacle pause time increase
* using distance to goal
* turn arcmove on/off with state rosarcmove true/false
* stop cancels recovery routine
* more arcmove, follow localpath AND global path
* Merge branch 'master' into devel
* arcmove tweaks
* arcmove testing
* add blank maps, oculusprimesocket module updates
* oculusprime module add reconnect
* oclusprimesocket module check for lost connection
* increase sleep on goal abort, change node name
* recovery rotation delay
* map node update
* move pwm set to java
* state name change to navsystemstatus
* node announce to java server
* drop first global path
* no rotate on initialposition if docked
* add global_planner_params.yaml
* forked depthimage_to_laserscan floor plane objects
* param tweaks
* multiple waypoint setting
* force remote map zoom on update during mapping
* odometry, gmapping improvements
* globalpath follow
* add laserscan data to remote map
* web remote nav global path added, critical odom bugfix
* web remote navigation beginnings
* write openni data to ram drive
* manifest: correct pkg names in run_depends
  Separator is an underscore, not a hyphen.
* updated readme
* updated runtime dependencies, readme, comments
* python module docs
* readme modify
* make_map.launch edited online with Bitbucket
* make_map.launch edited online with Bitbucket
* README.md edited online with Bitbucket
* README.md edited online with Bitbucket
* README.md edited online with Bitbucket
* README.md edited online with Bitbucket
* cleanup, make_map.launch streamlined
* no telnet login reuired
* increase sim_time to increase overall speed
* added lag subtract more accurate odom
* added after-goal-rotate delay, tested nav_test.py OK
* added delay after initial turn
* was using wrong plan, switch to /move_base/DWAPlannerROS/global_plan
* dwa global path initial turn, better
* dwa turn towards global path
* dwa best yet
* dwa working almost
* dwa base controller testing
* modified for indigo
* hydro working ok, pre indigo dev
* odom-map tf goal pose working ok
* odom-map tf goal pose, issues
* goal pose monitoring works as ong as odo solid, need odom-map-tf monitoring instead
* slower turning, still working ok
* added goal status monitoring
* added goal pose for final orientation, sort of working OK
* trying new base controller that follows local path
* testing recording of cmd_vel, derive vector
* added move buffer to base controller, doesn't work well
* initial import
* Contributors: G.A. vd. Hoorn, colin, skyzorg, xaxxon
