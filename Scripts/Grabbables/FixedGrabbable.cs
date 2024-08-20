using Godot;
using System;

public partial class FixedGrabbable : Grabbable
{
    [Export] public bool LeftHanded;

    public override bool Grab(PhysbodyHand Hand)
    {
        return Hand.IsLeftHanded == LeftHanded;
    }
    public override Transform3D CalculateGrabPose(PhysbodyHand Hand)
    {
        return ParentRigidBody.GlobalTransform.Inverse() * GlobalTransform;
    }
}