using Godot;
using System;

public partial class CircleGrabbable : Grabbable
{
    [Export] public float Radius;

    public override Transform3D CalculateGrabPose(PhysbodyHand Hand)
    {
        Vector3 localPalmPoint = GlobalTransform.Inverse() * Hand.PalmGrabPoint.GlobalPosition;
        Vector3 projectedPalmPoint = new Vector3(localPalmPoint.X, 0, localPalmPoint.Z);
        projectedPalmPoint = projectedPalmPoint.LimitLength(Radius);

        Vector3 localPalmUp = GlobalBasis.Inverse() * Hand.PalmGrabPoint.GlobalBasis * Vector3.Up;
        Vector3 projectedUp = new Plane(Vector3.Up).Project(localPalmUp);

        Basis grabOrientation = Basis.LookingAt(Vector3.Down, projectedUp);

        return ParentRigidBody.GlobalTransform.Inverse() * GlobalTransform * new Transform3D(grabOrientation, projectedPalmPoint);
    }
}
