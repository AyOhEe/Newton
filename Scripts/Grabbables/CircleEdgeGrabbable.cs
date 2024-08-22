using Godot;
using System;

public partial class CircleEdgeGrabbable : Grabbable
{
    [Export] public Node3D GrabOrientationOffset;
    [Export] public float Radius;
    [Export] public bool PreferRotationPose;

    private Basis _offset => GrabOrientationOffset == null ? Basis.Identity : GrabOrientationOffset.Basis;

    public override Transform3D CalculateGrabPose(PhysbodyHand Hand)
    {
        Vector3 localPalmPoint = GlobalTransform.Inverse() * Hand.PalmGrabPoint.GlobalPosition;
        Basis localPalmBasis = GlobalBasis.Inverse() * Hand.PalmGrabPoint.GlobalBasis;

        Vector3 projectedPalmPoint = new Vector3(localPalmPoint.X, 0, localPalmPoint.Z);
        Vector3 radiusDirection = CalculateRadiusDirection(localPalmBasis, projectedPalmPoint);
        Vector3 grabPoint = radiusDirection * Radius;


        Basis counterRotatedBasis = localPalmBasis * _offset.Inverse();
        Vector3 grabUp = Vector3.Up.Cross(radiusDirection).Normalized();
        Vector3 counterRotatedUp = counterRotatedBasis * Vector3.Up;
        if(counterRotatedUp.Dot(grabUp) < 0)
        {
            grabUp = -grabUp;
        }

        Basis grabOrientation = Basis.LookingAt(-radiusDirection, grabUp) * _offset;


        return ParentRigidBody.GlobalTransform.Inverse() * GlobalTransform * new Transform3D(grabOrientation, grabPoint);
    }

    private Vector3 CalculateRadiusDirection(Basis localPalmBasis, Vector3 projectedPalmPoint)
    {
        if (PreferRotationPose)
        {
            return new Plane(Vector3.Up).Project(localPalmBasis * Vector3.Back).Normalized();
        }
        else
        {
            return projectedPalmPoint.Normalized();
        }
    }
}