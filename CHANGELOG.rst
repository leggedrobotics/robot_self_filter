^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robot_self_filter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2026-07-20)
------------------
* Optimize box ray intersections and reuse ray-hit storage.
* Add per-body bounding-sphere culling for containment checks.
* Add deterministic correctness tests, a reproducible benchmark, and an installed-node smoke test.
* Preserve organized-cloud metadata and initialize removed custom point fields deterministically.
* Respect ``use_sim_time`` overrides and ``max_queue_size`` at runtime.
* Remove an unused duplicate TF listener that could corrupt shutdown during rapid SIGINT.
* Bound per-cloud diagnostics and skip collision-marker work without subscribers.
* Publish collision markers for every supported LiDAR point type.
* Support prefixed TF frame names while matching their suffix to URDF links.
* Validate mesh topology, resource sizes, and STL file sizes before conversion.
* Declare and export direct Jazzy dependencies and verify downstream consumption.
* Contributors: Lorenzo Terenzi

1.0.0 (2025-08-19)
------------------
* First stable ROS 2 Humble source release.
* Add support for generic, Ouster, Hesai, Robosense, and Pandar point clouds.
* Add configurable per-shape scaling and padding plus collision-shape visualization.
* Contributors: Lorenzo Terenzi

0.1.30 (2017-01-20)
-------------------
* Fix typo in CMakeLists.txt: CATKIN-DEPENDS -> CATKIN_DEPENDS
* Add ~max_queue_size parameter for subscription queue size
* Contributors: Devon Ash, Kentaro Wada, Ryohei Ueda

0.1.29 (2015-12-05)
-------------------
* pr2_navigation_self_filter -> robot_self_filter
* Add robot_self_filter namespace before bodies and shapes namespace.
  geometric_shapes package also provides bodies and shapes namespace
  and same classes and functions. If a program is linked with
  geometric_shapes and robot_self_filter, it may cause strange behavior
  because of symbol confliction.
* Contributors: Ryohei Ueda

0.1.28 (2015-12-04)
-------------------
* Added indigo devel
* Set correct timestamp for self filtered cloud
  This is needed because pcl drops some value of timestamp.
  So pcl::fromROSMsg and pcl::toROSMsg does not work to get correct timestamp.
  Protected member variables in SelfMask for subclass of SelfMask
* Protected member variables in SelfMask for subclass of SelfMask
* Contributors: Devon Ash, Kentaro Wada, Ryohei Ueda, TheDash

0.1.27 (2015-12-01)
-------------------
* Porting robot_self_filter from pr2_navigation_self_filter
* Initial commit
* Contributors: Devon Ash, Ryohei Ueda
