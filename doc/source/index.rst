.. _tabletop:

object_recognition_tabletop: Tabletop Object Recognition
========================================================

Tabletop is a port of the method in http://www.ros.org/wiki/tabletop_object_detector originally developed by Marius Muja from the FLANN TODO fame.

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

.. toggle:: Non-ROS

    .. code-block:: sh

        rosrun object_recognition_ros server -c `rospack find object_recognition_tabletop`/conf/detection.table.ork




TODO create a config file that just displays the clustertop cell + create the corresponding cell

TODO detail the topics

TODO add an RViz screenshot

Object Finder
-------------

The part of the pipeline that recognizes objects functions as follows: it takes the clusters segmented from the previous stage, finds possibile candidates in the database and then tries to match their meshes to the observed clusters during the final ICP step.

The objects are assumed to be rotationnally symmetric and flat on the table: the ICP step is therefore performed only on the x,y position of the object. No color information is used.