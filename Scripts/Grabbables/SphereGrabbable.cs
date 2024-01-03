using Godot;
using System;

public partial class SphereGrabbable : Grabbable
{
    [ExportCategory("Sphere Grabbable Settings")]
    [Export] private float _Radius = 0.5f;

    public override Transform3D CalculateGrabPose(PhysbodyHand Hand)
    {
        Vector3 localPalmPoint = GlobalTransform.Inverse() * Hand.PalmGrabPoint.GlobalPosition;
        Vector3 projectedPalmPoint = localPalmPoint.Normalized() * _Radius;

        Vector3 worldspacePoint = GlobalTransform * projectedPalmPoint;
        Vector3 parentspacePoint = ParentRigidBody.GlobalTransform.Inverse() * worldspacePoint;

        Vector3 localRight = GlobalBasis.Inverse() * Hand.PalmGrabPoint.GlobalBasis * Vector3.Right;
        Vector3 projectedRight = new Plane(projectedPalmPoint).Project(localRight);
        Vector3 projectedUp = projectedRight.Cross(-projectedPalmPoint);
        Basis worldspaceBasis = GlobalBasis * Basis.LookingAt(-projectedPalmPoint, projectedUp);
        Basis parentspaceBasis = ParentRigidBody.GlobalBasis.Inverse() * worldspaceBasis;

        return new Transform3D(parentspaceBasis, parentspacePoint);
    }
}
