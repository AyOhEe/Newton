using Godot;
using Godot.Collections;
using System;
using System.Linq;

public partial class PhysbodyHand : RigidBody3D
{
    [Export] public bool IsLeftHanded { get; private set; }
    [Export] public Node3D PalmGrabPoint { get; private set; }

    //[ExportCategory("Grab Joint settings")]
    


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

        //calculate the position to feed into the joint
        Transform3D parentspaceHand = G.ParentRigidBody.GlobalTransform.Inverse() * GlobalTransform;
        Vector3 grabPosition = parentspacePose.Origin - parentspaceHand.Origin;


        //configure the joint
        _HeldGrabbable.ParentRigidBody.AddChild(_HeldJoint);
        _HeldJoint.NodeA = _HeldGrabbable.ParentRigidBody.GetPath();
        _HeldJoint.NodeB = this.GetPath();

        _HeldJoint.SetParamX(Generic6DofJoint3D.Param.LinearLowerLimit, grabPosition.X);
        _HeldJoint.SetParamX(Generic6DofJoint3D.Param.LinearUpperLimit, grabPosition.X);
        _HeldJoint.SetParamY(Generic6DofJoint3D.Param.LinearLowerLimit, grabPosition.Y);
        _HeldJoint.SetParamY(Generic6DofJoint3D.Param.LinearUpperLimit, grabPosition.Y);
        _HeldJoint.SetParamZ(Generic6DofJoint3D.Param.LinearLowerLimit, grabPosition.Z);
        _HeldJoint.SetParamZ(Generic6DofJoint3D.Param.LinearUpperLimit, grabPosition.Z);
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
