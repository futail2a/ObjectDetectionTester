#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

"""
 @file TrajectoryPlanner_idl_examplefile.py
 @brief Python example implementations generated from TrajectoryPlanner.idl
 @date $Date$


"""

import omniORB
from omniORB import CORBA, PortableServer
import Manipulation, Manipulation__POA
import RTC

class ObjectDetectionService_i (Manipulation__POA.ObjectDetectionService):
    """
    @class ObjectDetectionService_i
    Example class implementing IDL interface Manipulation.ObjectDetectionService
    """

    def __init__(self):
        """
        @brief standard constructor
        Initialise member variables here
        """
        pass

    # ReturnValue detectObject(in ObjectIdentifier objectID, out ObjectInfo objInfo)
    def detectObject(self, objectID):
        objInfo = Manipulation.ObjectInfo(Manipulation.ObjectIdentifier(objectID.name),RTC.Pose3D(RTC.Point3D(2.0,2.0,2.0), RTC.Orientation3D(2.0,2.0,2.0)))

        objInfo.pose.position.x =0.40
        objInfo.pose.position.y =0.0
        objInfo.pose.position.z =0.62
        objInfo.pose.orientation.p =1.56#118.0
        objInfo.pose.orientation.r =3.14#1.54#0.0
        objInfo.pose.orientation.y =3.14#2.717      

	print isinstance(objInfo, Manipulation.ObjectInfo)
        print objInfo

        result = Manipulation.ReturnValue(Manipulation.OK,"test")
        return (result, objInfo)

    # ReturnValue setBaseFrame(in Matrix34 frame)
    def setBaseFrame(self, frame):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result



class ObjectHandleStrategyService_i (Manipulation__POA.ObjectHandleStrategyService):
    """
    @class ObjectHandleStrategyService_i
    Example class implementing IDL interface Manipulation.ObjectHandleStrategyService
    """

    def __init__(self):
        """
        @brief standard constructor
        Initialise member variables here
        """
        pass

    # ReturnValue getApproachOrientation(in ObjectInfo objInfo, out EndEffectorPose eePos)
    def getApproachOrientation(self, objInfo):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result, eePos



class KinematicSolverService_i (Manipulation__POA.KinematicSolverService):
    """
    @class KinematicSolverService_i
    Example class implementing IDL interface Manipulation.KinematicSolverService
    """

    def __init__(self):
        """
        @brief standard constructor
        Initialise member variables here
        """
        pass

    # ReturnValue solveKinematics(in EndEffectorPose targetPose, in JointAngleSeq startJointAngles, out JointAngleSeq targetJointAngles)
    def solveKinematics(self, targetPose, startJointAngles):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result, targetJointAngles



class CollisionDetectionService_i (Manipulation__POA.CollisionDetectionService):
    """
    @class CollisionDetectionService_i
    Example class implementing IDL interface Manipulation.CollisionDetectionService
    """

    def __init__(self):
        """
        @brief standard constructor
        Initialise member variables here
        """
        pass

    # ReturnValue isCollide(in RobotIdentifier robotID, in JointAngleSeq jointAngles, out CollisionPairSeq collisions)
    def isCollide(self, robotID, jointAngles):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result, collisions



class ManipulationPlannerService_i (Manipulation__POA.ManipulationPlannerService):
    """
    @class ManipulationPlannerService_i
    Example class implementing IDL interface Manipulation.ManipulationPlannerService
    """

    def __init__(self):
        """
        @brief standard constructor
        Initialise member variables here
        """
        pass

    # ReturnValue planManipulation(in RobotIdentifier robotID, in JointAngleSeq startJointAngles, in JointAngleSeq goalJointAngles, out ManipulationPlan manipPlan)
    def planManipulation(self, robotID, startJointAngles, goalJointAngles):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result, manipPlan



class ModelServerService_i (Manipulation__POA.ModelServerService):
    """
    @class ModelServerService_i
    Example class implementing IDL interface Manipulation.ModelServerService
    """

    def __init__(self):
        """
        @brief standard constructor
        Initialise member variables here
        """
        pass

    # ReturnValue getModelInfo(in RobotIdentifier robotID, out RobotJointInfo jointsInfo)
    def getModelInfo(self, robotID):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result, jointsInfo

    # ReturnValue getMeshInfo(in RobotIdentifier robotID, out MeshInfo mesh)
    def getMeshInfo(self, robotID):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result, mesh



class MotionGeneratorService_i (Manipulation__POA.MotionGeneratorService):
    """
    @class MotionGeneratorService_i
    Example class implementing IDL interface Manipulation.MotionGeneratorService
    """

    def __init__(self):
        """
        @brief standard constructor
        Initialise member variables here
        """
        pass

    # ReturnValue followManipPlan(in ManipulationPlan manipPlan)
    def followManipPlan(self, manipPlan):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

    # ReturnValue getCurrentRobotJointAngles(out JointAngleSeq jointAngles)
    def getCurrentRobotJointAngles(self):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result, jointAngles


if __name__ == "__main__":
    import sys
    
    # Initialise the ORB
    orb = CORBA.ORB_init(sys.argv)
    
    # As an example, we activate an object in the Root POA
    poa = orb.resolve_initial_references("RootPOA")

    # Create an instance of a servant class
    servant = ObjectDetectionService_i()

    # Activate it in the Root POA
    poa.activate_object(servant)

    # Get the object reference to the object
    objref = servant._this()
    
    # Print a stringified IOR for it
    print orb.object_to_string(objref)

    # Activate the Root POA's manager
    poa._get_the_POAManager().activate()

    # Run the ORB, blocking this thread
    orb.run()

