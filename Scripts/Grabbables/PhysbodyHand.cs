using Godot;
using Godot.Collections;
using System;
using System.Linq;

public partial class PhysbodyHand : RigidBody3D
{
    [Export] public bool IsLeftHanded { get; private set; }
    [Export] public Node3D PalmGrabPoint { get; private set; }


    private Array<Grabbable> _NearbyGrabbables = new Array<Grabbable>();
    private Grabbable _HeldGrabbable = null;
    private GrabbableJoint _HeldJoint = null;



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
            _HeldGrabbable.SetToolPrimary(0);
            _HeldGrabbable.SetToolSecondary(0);
            _HeldGrabbable = null;

            _HeldJoint.HandleDestruction();
            _HeldJoint.QueueFree();
            _HeldJoint = null;
        }
    }


    private void GrabObject(Grabbable G, Transform3D parentspacePose)
    {
        //create the joint and store references for this grab
        _HeldGrabbable = G;
        _HeldGrabbable.SetToolPrimary(0);
        _HeldGrabbable.SetToolSecondary(0);
        _HeldJoint = new GrabbableJoint(this, _HeldGrabbable.ParentRigidBody);

        //configure the joint's position and rotation
        ConfigureHeldJointRotation(G, parentspacePose);
        ConfigureHeldJointPosition(G, parentspacePose);

        //add the joint to the scene tree
        _HeldGrabbable.ParentRigidBody.AddChild(_HeldJoint);
    }
    private void ConfigureHeldJointPosition(Grabbable G, Transform3D parentspacePose)
    {
        _HeldJoint.TargetPosition = PalmGrabPoint.Position + (_HeldJoint.TargetRotation * -parentspacePose.Origin);

    }
    private void ConfigureHeldJointRotation(Grabbable G, Transform3D parentspacePose)
    {
        _HeldJoint.TargetRotation = PalmGrabPoint.Basis * parentspacePose.Basis.Inverse();

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
        if (name == "by_button")
        {
            _HeldGrabbable?.SetToolSecondary(1);
        }
    }
    public void OnButtonReleased(string name)
    {
        if (name == "grip_click")
        {
            OnGrabExit();
        }
        if (name == "by_button")
        {
            _HeldGrabbable?.SetToolSecondary(0);
        }
    }
    public void OnFloatChanged(string name, float value)
    {
        if (name == "trigger") //TODO deadzone?
        {
            _HeldGrabbable?.SetToolPrimary(value);
        }
    }
}
