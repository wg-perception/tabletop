Forthcoming
-----------
* Merge pull request `#21 <https://github.com/wg-perception/tabletop/issues/21>`_ from hris2003/master
  Complete explanation about tabletop
* Complete explanation about tabletop
* remove tabletop_db dependency
* remove any reference to the household database
  Everything is being moved to object_recognition_ros_tabletop
* add more examples for `#13 <https://github.com/wg-perception/tabletop/issues/13>`_
* remove useless inclusion
* remove useless household database code
* get the code to compile on Indigo
* Merge pull request `#15 <https://github.com/wg-perception/tabletop/issues/15>`_ from shadow-robot/Fix_object_pose
  Fix object pose by removing redundant z_min correction.
* Merge pull request `#16 <https://github.com/wg-perception/tabletop/issues/16>`_ from cottsay/master
  build_depend on assimp-dev instead of assimp
* build_depend on assimp-dev instead of assimp
* Fix object pose by removing redundant z_min correction.
* Merge pull request `#14 <https://github.com/wg-perception/tabletop/issues/14>`_ from shadow-robot/fix_cloud_frame
  Fix cloud frame
* Cosmetic change.
* Fix result point cloud. The points are trasformed back to their original reference.
* Merge pull request `#12 <https://github.com/wg-perception/tabletop/issues/12>`_ from awesomebytes/database_id_fix
  Fix so the detection can say which object was correctly detected.
* Fix so the detection can say which object was correctly detected.
* build assimp 3 no matter what
* fixes part of `#11 <https://github.com/wg-perception/tabletop/issues/11>`_
* simplify the vector handling
* Merge pull request `#10 <https://github.com/wg-perception/tabletop/issues/10>`_ from bmagyar/patch-1
  Fix comments of class members
* Fix comments of class members
  to avoid confusion
* Contributors: Bence Magyar, Ha Dang, Sammy Pfeiffer, Scott K Logan, Toni Oliver, Vincent Rabaud

0.3.1 (2013-12-18  21:17:06 +0100)
----------------------------------
- improve accuracy by better sampling the mesh for ICP
- allow the pipeline to load meshes from an ORK DB (not only household anymore)

0.3.0 (2013-12-08  19:17:06 +0100)
----------------------------------
- move the visualization of the Table to object_recognition_ros
