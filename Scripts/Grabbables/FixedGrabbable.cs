using Godot;
using System;

public partial class FixedGrabbable : Grabbable
{
    public override Transform3D CalculateGrabPose(PhysbodyHand Hand)
    {
        throw new NotImplementedException();
    }
}