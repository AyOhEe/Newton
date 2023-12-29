using Godot;
using Godot.Collections;
using System;
using System.Linq;

public partial class PhysbodyHand : RigidBody3D
{
    [Export] public bool IsLeftHanded { get; private set; }
    [Export] public Node3D PalmGrabPoint { get; private set; }


    [ExportCategory("Grab Joint settings")]
    [Export] private float _LinearSpringStiffness = 1000;
    [Export] private float _LinearSpringDamping = 100;



    private Array<Grabbable> _NearbyGrabbables = new Array<Grabbable>();
    private Grabbable _HeldGrabbable = null;
    private Generic6DofJoint3D _HeldJoint = null;



    public void IntroduceGrabbable(Grabbable G)
    {
        _NearbyGrabbables.Add(G);
    }
    public void RemoveGrabbable(Grabbable G)
    {
        //this only returns false if the item doesn't exist. no exceptions thrown!
        _NearbyGrabbables.Remove(G);
    }


    private void OnGrabStart()
    {
        //if we're already holding something, don't bother checking for another thing
        if (_HeldGrabbable != null)
        {
            return;
        }
        //if there's nothing to grab, don't try to
        if (_NearbyGrabbables.Count == 0)
        {
            return;
        }


        //find the grab pose closest to the current pose of the hand
        Grabbable nearestGrabbable = _NearbyGrabbables[0];
        Transform3D nearestPose = _NearbyGrabbables[0].CalculateGrabPose(this);
        float lowestCost = CalculatePoseCost(_NearbyGrabbables[0].ParentRigidBody.GlobalTransform * nearestPose);

        foreach(Grabbable G in _NearbyGrabbables.Skip(1))
        {
            Transform3D pose = G.CalculateGrabPose(this);
            float cost = CalculatePoseCost(G.ParentRigidBody.GlobalTransform * pose);

            if (cost < lowestCost)
            {
                nearestGrabbable = G;
                lowestCost = cost;
                nearestPose = pose;
            }
        }

        //grab that grabbable at that position
        if (nearestGrabbable.Grab(this))
        {
            //grab accepted, create the joint
            GrabObject(nearestGrabbable, nearestPose);
        }
    }
    private void OnGrabExit()
    {
        //release what we're holding, if anything, and if we're allowed to
        if(_HeldGrabbable != null && _HeldGrabbable.Release(this))
        {
            _HeldGrabbable = null;

            _HeldJoint.QueueFree();
            _HeldJoint = null;
        }
    }


    private void GrabObject(Grabbable G, Transform3D parentspacePose)
    {
        //create the joint and store references for this grab
        _HeldJoint = new Generic6DofJoint3D();
        _HeldGrabbable = G;



        //configure the joint's bodies, position and rotation
        _HeldJoint.NodeA = _HeldGrabbable.ParentRigidBody.GetPath();
        _HeldJoint.NodeB = this.GetPath();
        ConfigureHeldJointPosition(G, parentspacePose);
        ConfigureHeldJointRotation(G, parentspacePose);

        //add the joint to the scene tree
        _HeldGrabbable.ParentRigidBody.AddChild(_HeldJoint);

    }
    private void ConfigureHeldJointPosition(Grabbable G, Transform3D parentspacePose)
    {
        //calculate the position to feed into the joint
        Vector3 parentspaceHandPos = G.ParentRigidBody.GlobalTransform.Inverse() * PalmGrabPoint.GlobalPosition;
        Vector3 grabPosition = -parentspaceHandPos + parentspacePose.Origin;

        //setting the linear limit to be at the grab position instead of the linear spring 
        //provides results that better keeps the hand attatched to the body
        _HeldJoint.SetParamX(Generic6DofJoint3D.Param.LinearLowerLimit, grabPosition.X);
        _HeldJoint.SetParamX(Generic6DofJoint3D.Param.LinearUpperLimit, grabPosition.X + 0.001f);
        _HeldJoint.SetParamY(Generic6DofJoint3D.Param.LinearLowerLimit, grabPosition.Y);
        _HeldJoint.SetParamY(Generic6DofJoint3D.Param.LinearUpperLimit, grabPosition.Y + 0.001f);
        _HeldJoint.SetParamZ(Generic6DofJoint3D.Param.LinearLowerLimit, grabPosition.Z);
        _HeldJoint.SetParamZ(Generic6DofJoint3D.Param.LinearUpperLimit, grabPosition.Z + 0.001f);

        //adding *some* spring does, however, do an even better job on top of that
        _HeldJoint.SetParamX(Generic6DofJoint3D.Param.LinearSpringStiffness, _LinearSpringStiffness);
        _HeldJoint.SetParamY(Generic6DofJoint3D.Param.LinearSpringStiffness, _LinearSpringStiffness);
        _HeldJoint.SetParamZ(Generic6DofJoint3D.Param.LinearSpringStiffness, _LinearSpringStiffness);
        _HeldJoint.SetParamX(Generic6DofJoint3D.Param.LinearSpringDamping, _LinearSpringDamping);
        _HeldJoint.SetParamY(Generic6DofJoint3D.Param.LinearSpringDamping, _LinearSpringDamping);
        _HeldJoint.SetParamZ(Generic6DofJoint3D.Param.LinearSpringDamping, _LinearSpringDamping);
    }
    private void ConfigureHeldJointRotation(Grabbable G, Transform3D parentspacePose)
    {
        //TODO this
    }
    private float CalculatePoseCost(Transform3D nearestPose)
    {
        return PalmGrabPoint.GlobalPosition.DistanceTo(nearestPose.Origin);
    }


    public void OnButtonPressed(string name)
    {
        if (name == "grip_click")
        {
            OnGrabStart();
        }
    }
    public void OnButtonReleased(string name)
    {
        if (name == "grip_click")
        {
            OnGrabExit();
        }
    }
}
