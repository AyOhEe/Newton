using Godot;
using System;

public partial class CylinderGrabbable : Grabbable
{
    [Export] public float Radius;
    [Export] public float Height;
    [Export] public bool PreferRotationPose;

    public override Transform3D CalculateGrabPose(PhysbodyHand Hand)
    {
        Vector3 localPalmPoint = GlobalTransform.Inverse() * Hand.PalmGrabPoint.GlobalPosition;
        Basis localPalmBasis = GlobalBasis.Inverse() * Hand.PalmGrabPoint.GlobalBasis;

        //projection on Vector3.Up is equivalent to taking only the Y component
        Vector3 projectedPalmPoint = new Vector3(0, localPalmPoint.Y, 0);
        //the cylinder is centered at the origin, therefore the extents are -height/2 and height/2
        Vector3 clampedPalmPoint = projectedPalmPoint.LimitLength(Height / 2);

        Vector3 radiusDirection = CalculateRadiusDirection(localPalmBasis, projectedPalmPoint, localPalmPoint);
        Vector3 grabPoint = clampedPalmPoint + (radiusDirection * Radius);


        //pick the most comfortable orientation, closest to the current hand orientation
        Vector3 grabUpVector = Vector3.Up;
        Vector3 localUpVector = GlobalBasis.Inverse() * Hand.PalmGrabPoint.GlobalBasis * Vector3.Up;
        if (grabUpVector.Dot(localUpVector) < 0)
        {
            grabUpVector = -grabUpVector;
        }

        Basis grabOrientation = Basis.LookingAt(-radiusDirection, grabUpVector);


        return ParentRigidBody.GlobalTransform.Inverse() * GlobalTransform * new Transform3D(grabOrientation, grabPoint);
    }

    private Vector3 CalculateRadiusDirection(Basis localPalmBasis, Vector3 projectedPalmPoint, Vector3 localPalmPoint)
    {
        if (PreferRotationPose)
        {
            return new Plane(Vector3.Up).Project(localPalmBasis * Vector3.Back).Normalized();
        }
        else
        {
            return projectedPalmPoint.DirectionTo(localPalmPoint);
        }
    }
}