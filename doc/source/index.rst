:orphan:

.. _tabletop:

object_recognition_tabletop: Tabletop Object Recognition
========================================================

Tabletop is a port of the method in http://www.ros.org/wiki/tabletop_object_detector originally developed by Marius Muja from the `FLANN <https://github.com/mariusmuja/flann>`_ fame.

This object detection method has two parts: a table finder and an object recognizer. The recognition part can only recognize objects that are rotationnally symmetric.

Table Finder
------------

The table finder finds planes in the scene and segments out the objects on top of it. This part of this method can be useful in itself if you need the clusters as a pre-processing step for other object recognition techniques or if raw clusters are enough for you (for robot grasping for example).

.. toggle_table::
    :arg1: Non-ROS
    :arg2: ROS

To launch the pipeline, simply execute execute the following

.. toggle:: Non-ROS

    .. code-block:: sh

        ./detection -c ${PATH_TO_YOUR_TABLETOP_FOLDER}/conf/detection.table.ork

.. toggle:: ROS

    .. code-block:: sh

        rosrun object_recognition_ros server -c `rospack find object_recognition_tabletop`/conf/detection.table.ork

    In ROS mode, several topics are published:

       * a MarkerArray.msg for the clusters found on the planes as ``/tabletop/clusters``
       * a Table.msg for the different tables (the RViz ork plugin can help visualize those)

Object Finder
-------------

The part of the pipeline that recognizes objects functions as follows: it takes the clusters segmented from the previous stage, finds possible candidates in the database and then tries to match their meshes to the observed clusters during the final ICP step.

The objects are assumed to be rotationnally symmetric and flat on the table: the ICP step is therefore performed only on the x,y position of the object. No color information is used.

Example
-------

Here is what the scene looks like:

.. image:: example1.png
   :width: 100%

The pipeline then finds the planes and the clusters on top of it:

.. image:: example2.png
   :width: 100%

And it then identifies the clusters as objects in the database:

.. image:: example3.png
   :width: 100%
