using Godot;
using System;

public partial class PlaneGrabbable : Grabbable
{
    [ExportCategory("Plane Grabbable Settings")]
    [Export] private Vector2 _Dimensions = Vector2.One;

    public override Transform3D CalculateGrabPose(PhysbodyHand Hand)
    {
        Vector3 localPalmPoint = GlobalTransform.Inverse() * Hand.PalmGrabPoint.GlobalPosition;
        Vector3 projectedPalmPoint = new Plane(Vector3.Up).Project(localPalmPoint);
        projectedPalmPoint.X = Mathf.Clamp(projectedPalmPoint.X, -_Dimensions.X / 2, _Dimensions.X / 2);
        projectedPalmPoint.Z = Mathf.Clamp(projectedPalmPoint.Z, -_Dimensions.Y / 2, _Dimensions.Y / 2);

        Vector3 worldspacePoint = GlobalTransform * projectedPalmPoint;
        Vector3 parentspacePoint = ParentRigidBody.GlobalTransform.Inverse() * worldspacePoint;

        Vector3 localRight = GlobalBasis.Inverse() * Hand.PalmGrabPoint.GlobalBasis * Vector3.Right;
        Vector3 projectedRight = new Plane(Vector3.Up).Project(localRight);
        Basis worldspaceBasis = GlobalBasis * Basis.LookingAt(Vector3.Down, projectedRight.Cross(Vector3.Down));
        Basis parentspaceBasis = ParentRigidBody.GlobalBasis.Inverse() * worldspaceBasis;

        return new Transform3D(parentspaceBasis, parentspacePoint);
    }
}
